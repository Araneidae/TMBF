% [p_out, m_out, pp] = fit_peaks(s, iq, [, 'noplot'] [, 'refine'] ...
%       [, 'count', count] [, 'threshold', threshold])
%
% Fits a sum of one-pole filters to the given (s, iq) data.  By default 3 peaks
% are fitted, and the data threshold for the initial peak fit is taken as 0.3.
%
% The model for each peak is  iq = a / (s - b)  and the overall model is a sum
% of these peaks.  The a coefficients are returned as
%
%   a = p_out(1:2:end)
%   b = p_out(2:2:end)
%
% The returned peak data p_out can be interpreted thus:
%
%   abs(a)
%       Magnitude of peak (in combination with peak width factor below).
%
%   angle(a)
%       Phase of peak.  For practical peak measurements we would expect all of
%       the phases to be constant, reflecting simply the filter's group delay.
%
%   real(b)
%       Frequence of the peak centre frequency
%
%   imag(b)
%       Peak width factor: larger numbers here correspond to wider peaks, and
%       the power under the peak is proportional to this value.
function [p_out, m_out, pp] = fit_peaks(s, iq, varargin) % count, threshold)
    params = parse_arguments(varargin);
    no_plot = params.no_plot;
    do_refine = params.do_refine;
    count = params.count;
    threshold = params.threshold;
    smoothing = params.smoothing;

    % Ensure both s and iq are column vectors
    s = s(:);
    iq = iq(:);

    % Perform initial fit using simple peak discovery and fitting.
    [pp, rr, iqrr] = quick_fit_peaks(s, iq, count, smoothing, threshold);

    % Refine the fit by full model error minimisation.  This adjusts the peak
    % parameters for a better fit.  Note that for small peaks this can make the
    % fit significantly worse.
    if do_refine
        p_out = refine_model(s, iq, flatten_model(pp));
    else
        p_out = flatten_model(pp);
    end
    m_out = eval_model(p_out, s);


    if ~no_plot
        plot_results(s, iq, pp, iqrr, p_out, m_out);
    end
end


% Refined argument parsing.
function params = parse_arguments(argsin)
    function [match, argsin] = pop(arg, argsin)
        match = false;
        for n = 1:length(argsin)
            if strcmp(arg, argsin{n})
                match = true;
                argsin(n) = [];
                return
            end
        end
    end

    % Consume the optional single word arguments first.
    [no_plot, argsin] = pop('noplot', argsin);
    [do_refine, argsin] = pop('refine', argsin);

    % Use the parser to consume the keyword arguments
    p = inputParser;
    addParamValue(p, 'count', 3);
    addParamValue(p, 'threshold', 0.3);
    addParamValue(p, 'smoothing', 16);
    parse(p, argsin{:});
    params = p.Results;

    params.no_plot = no_plot;
    params.do_refine = do_refine;
end


% Find peaks using smoothed second derivative and then use direct fit for each
% peak found.  Most of the time the result of this is good enough.
function [pp, rr, iqrr] = quick_fit_peaks(s, iq, count, smoothing, threshold)
    [rr, d2h] = find_peaks(iq, count, smoothing, threshold);

    pp = {};
    iqr = iq;
    iqrr = {};
    for n = 1:count
        [pp{n}, iqr] = fit_one_peak(s, iqr, rr{n});
        iqrr{n} = iqr;
        pp{n}.d2h = d2h(n);
    end
end


% Returns list of peaks as a list of ranges to fit.
function [rr, d2h] = find_peaks(iq, count, smoothing, threshold)
    % Do the filtering in the same rough and ready way as the IOC.
    aiq = abs(iq);
    smoothed_power = sum(reshape(aiq.^2, smoothing, []));
    dd = conv(smoothed_power, [1 -2 1], 'same');
    dd([1 end]) = 0;    % Knock out invalid end points

    rr = {};
    d2h = zeros(count, 1);
    for n = 1:count
        [rr{n}, dd, d2h(n)] = ...
            find_one_peak(aiq, smoothed_power, dd, smoothing, threshold);
    end
end


% Discovers a single peak by searching computed second derivative dd,
% eliminates the discovered peak from further matching, and computes a range
% over which to fit the peak.
function [full_range, dd, m] = find_one_peak(aiq, fp, dd, smoothing, threshold)
    [m, ix] = min(dd);

    left = ix;
    while left > 1 && fp(left-1) >= fp(left); left = left - 1; end
    while left > 1 && fp(left-1) <= fp(left); left = left - 1; end

    limit = length(fp);
    right = ix;
    while right < limit && fp(right+1) >= fp(right); right = right + 1; end
    while right < limit && fp(right+1) <= fp(right); right = right + 1; end

    dd(left:right) = 0;

    left_right = smoothing*(left - 1) + 1 : smoothing*right;
    max_p = max(aiq(left_right));
    full_range = find(aiq(left_right) >= threshold * max_p) + left_right(1) - 1;
end


% Initial peak fit.  First find peak by rough detection of data maximum, and
% then slice out surrounding peak for subsequent fitting.
function [result, iq_out] = fit_one_peak(s, iq, range)
    sr = s(range);
    iqr = iq(range);
    p = fit_peak(sr, iqr, ones(length(range), 1));

    % Fit the peak twice.  Use the first peak to compute weights to refine
    % results for the second final fit.
    p = fit_peak(sr, iqr, 1 ./ abs(sr - p(2)).^2);

    m = p(1) ./ (s - p(2));
    iq_out = iq - m;

    result = {};
    result.iq = iq;
    result.p = p;
    result.r = range;
    result.m = m;

    % Peak quality estimate: mean error scaled by peak height.
    result.q = mean(abs(m(range) - iq(range)).^2) / abs(p(1)) * -imag(p(2));
end


% Computes parameters (a, b) for best fit of model z = a/(f-b) weighted by w.
function p = fit_peak(f, z, w)
    z2 = z.*conj(z);
    sw = sum(w);
    sz = sum(w.*z);
    sz2 = sum(w.*z2);
    swz = sum(w.*f.*z);
    swz2 = sum(w.*f.*z2);

    M = [sw sz; conj(sz) sz2];
    V = [swz; swz2];
    p = M\V;
end


function model = flatten_model(pp)
    model = zeros(2 * length(pp), 1);
    for n = 1:length(pp)
        model(2*n - 1 : 2*n) = pp{n}.p;
    end
end


% Uses Levenberg-Marquardt minimisation to refine the fit.  We scale the error
% by the original model to emphasise those parts of the original fit which seem
% more promising.
function p_out = refine_model(s, iq, p_in)

    function e = eval_error(m)
        e = weights .* (eval_model(m, s) - iq);
    end

    function de = eval_error_d(m)
        de = zeros(size(s, 1), length(m));
        n = 0;
        for p = reshape(m, 2, [])
            q = 1 ./ (s - p(2));
            de(:, n + 1) = q;
            de(:, n + 2) = p(1) .* q.^2;
            n = n + 2;
        end

        de = repmat(weights, 1, length(m)) .* de;
    end

    % Compute weights to focus on the located peaks.
    weights = zeros(size(s));
    for p = reshape(p_in, 2, [])
        weights = weights + 1 ./ (s - p(2)).^2;
    end

    p_out = levmar_core(@eval_error, @eval_error_d, p_in, 1e-3);
end


function x = eval_model(m, s)
    x = zeros(size(s));
    for p = reshape(m, 2, [])
        x = x + p(1) ./ (s - p(2));
    end
end


% Slightly refined Levenberg-Marquardt fitting algorithm.  Initial value a is an
% Nx1 column vector and fdf returns an Mx1 column vector of errors and an MxN
% matrix of derivatives.
function a = levmar_core(f, df, a, ftol, lambda, maxiter, max_lambda)
    if ~exist('lambda', 'var');     lambda = 1e-3; end
    if ~exist('maxiter', 'var');    maxiter = 10; end
    if ~exist('max_lambda', 'var'); max_lambda = 1e8; end
    if ~exist('ftol', 'var');       ftol = 1e-6; end

    e = f(a);
    chi2 = e'*e;

    for s = 1:maxiter
        e = f(a);
        de = df(a);
        beta   = de' * e;
        alpha0 = de' * de;

        while true
            alpha = alpha0 + lambda * diag(diag(alpha0));
            a_new = a - alpha\beta;

            e = f(a_new);
            chi2_new = e'*e;
            if chi2_new > chi2
                % Increase the linear part of the fit and try again
                lambda = lambda * 10;
                assert(lambda < max_lambda);
            elseif chi2 - chi2_new < ftol * chi2_new
                % We're not improving enough to carry on
                a = a_new;
                return
            else
                break
            end
        end

        a = a_new;
        lambda = lambda / 10;
        chi2 = chi2_new;
    end
end


% function plot_results(s, iq, p1, p2, p3, iq1, iq2, p_out, m_out)
function plot_results(s, iq, pp, iqrr, p_out, m_out)
    function c = colour(n)
        colours = 'brmgcky';
        c = colours(mod(n - 1, length(colours)) + 1);
    end


    initial_model = zeros(size(s));
    for p = pp
        initial_model = initial_model + p{1}.m;
    end

    r = 0.3;
    x_lim = [1-r r; r 1-r] * [s(1); s(end)];

    figure(1); clf;

    subplot 221
    hold all
    for n = 1:length(pp)
        p = pp{n};
        plot( ...
            s, abs(p.m), ['--' colour(n)], ...
            s, abs(p_out(2*n-1)./(s-p_out(2*n))), colour(n));
    end

    subplot 222
    hold all
    strings = {};
    for n = 1:length(pp)
        p = pp{n};
        plot(p.iq(p.r), ['.' colour(n)]);
        plot(p.m(p.r), colour(n));
        strings{2*n-1} = sprintf('P%d data', n);
        strings{2*n} = sprintf('P%d fit', n);
    end
    legend(strings)
    axis equal

    subplot 223
    semilogy(s, abs(iq), ':', s, abs(m_out))

    subplot 224
    plot(real(iq), imag(iq), ':', ...
        real(initial_model), imag(initial_model), '--', ...
        real(m_out), imag(m_out));
    legend('Raw data', 'Initial fit', 'Final fit')
    axis equal
end

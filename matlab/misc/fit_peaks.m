% [p_out, detail] = fit_peaks(s, iq, varargin)
%
% The following arguments can be passed as varargin (with defaults shown for
% arguments taking parameters):
%   'noplot'            Suppresses plotting of results
%   'refit'             Refine linear fit by refitting with weights
%   'repeat'            Repeat linear fit with corrected data and weigths
%   'lmrefine'          Triggers post-fit non-linear refinement
%   'count', 3          Specify number of peaks to fit
%   'threshold', 0.3    Height threshold for selecting points to include in fit
%   'smoothing', 16     Smoothing factor (must be power of 2)
%   'figure', 1         Figure to use for plotting results
%   'xlim', []          Zoom in to range for plotted results
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
%   -imag(b)
%       Peak width factor: larger numbers here correspond to wider peaks, and
%       the power under the peak is proportional to this value.
function [p_out, detail] = fit_peaks(s, iq, varargin)
    % Parse optional parameters
    params = parse_arguments(varargin);
    count = params.count;

    % Ensure both s and iq are column vectors
    s = s(:);
    iq = iq(:);

    % Perform initial fit using simple peak discovery and fitting.
    detail = fit_all_peaks( ...
        s, iq, count, params.smoothing, params.threshold, params.refit);

    if params.repeat
        detail = repeat_fit(s, iq, count, detail.r, detail);
    end

    % Refine the fit by full model error minimisation.  This adjusts the peak
    % parameters for a better fit.  Note that for small peaks this can make the
    % fit significantly worse.
    p_out = detail.p;
    if params.lmrefine
        p_out = refine_model(s, iq, p_out);
    end
    detail.m_out = eval_model(p_out, s);

    % Show results
    if ~params.noplot
        plot_results(s, iq, count, detail, p_out, params.figure, params.xlim);
    end
end


% Refined argument parsing.
function params = parse_arguments(argsin)
    function [match, argsin] = pop(arg, argsin)
        match = false;
        for m = 1:length(argsin)
            if strcmp(arg, argsin{m})
                match = true;
                argsin(m) = [];
                break
            end
        end
    end

    flag_args = {'noplot', 'refit', 'repeat', 'lmrefine'};

    % Consume single word arguments before using the parser to consume the
    % keyword value pair arguments.
    sN = length(flag_args);
    flags = [];
    for n = 1:sN
        [flags(n), argsin] = pop(flag_args{n}, argsin);
    end

    % Use the parser to consume the keyword arguments
    p = inputParser;
    addParamValue(p, 'count', 3);
    addParamValue(p, 'threshold', 0.3);
    addParamValue(p, 'smoothing', 16);
    addParamValue(p, 'figure', 1);
    addParamValue(p, 'xlim', []);
    parse(p, argsin{:});
    params = p.Results;

    % Convert all the single word arguments into returned attributes
    for n = 1:sN
        params = setfield(params, flag_args{n}, flags(n));
    end
end


% Find peaks using smoothed second derivative and then use direct fit for each
% peak found.  Most of the time the result of this is good enough.
function pp = fit_all_peaks(s, iq, count, smoothing, threshold, refit)
    [rr, d2h, bg] = find_peaks(iq, count, smoothing, threshold);

    pp = {};
    pp.r = rr;
    pp.d2h = d2h;
    pp.bg = bg;
    pp.iq = zeros(size(iq, 1), count);
    pp.p = {};
    pp.p = zeros(2 * count, 1);
    pp.m = zeros(size(iq, 1), count);
    pp.e = zeros(1, count);
    pp.n = zeros(1, count);

    iqr = iq;       % Residual IQ for fitting
    for n = 1:count
        pp.iq(:, n) = iqr;
        [p, m, e] = fit_one_peak(s, iqr, rr{n}, refit);
        % Compute residual IQ for next fit by subtracting model from data
        iqr = iqr - m;

        pp.m(:, n) = m;
        pp.p(2*n-1:2*n) = p;
        pp.e(n) = e;
        pp.n(n) = length(rr{n});
    end
end


% Repeats peak fit using existing model
function detail = repeat_fit(s, iq, count, ranges, detail)
    fit = reshape(detail.p, 2, []);
    for n = 1:count
        r = ranges{n};
        sr = s(r);
        iqr = iq(r);
        for m = 1:count
            if m ~= n
                iqr = iqr - fit(1,m)./(sr - fit(2,m));
            end
        end
        weights = 1./abs(sr - fit(2,n)).^2;
        fit(:,n) = fit_peak(sr, iqr, weights);

        % Recompute confidence error
        model = fit(1,n) ./ (sr - fit(2,n));
        detail.e(n) = mean((abs(iqr ./ model) - 1).^2);
    end
    detail.p = reshape(fit, [], 1);
end


% Returns list of peaks as a list of ranges to fit.
function [rr, d2h, background] = find_peaks(iq, count, smoothing, threshold)
    % Do the filtering in the same rough and ready way as the IOC.
    aiq = abs(iq);
    smoothed_power = sum(reshape(aiq.^2, smoothing, []));
    dd = conv(smoothed_power, [1 -2 1], 'same');
    dd([1 end]) = 0;    % Knock out invalid end points

    rr = {};
    d2h = zeros(count, 1);
    knockout_len = 0;
    for n = 1:count
        [rr{n}, dd, d2h(n), len] = ...
            find_one_peak(aiq, smoothed_power, dd, smoothing, threshold);
        knockout_len = knockout_len + len;
    end

    % Scale detected 2d peaks by residual rms background.
    background = sqrt(sum(dd.^2) / (length(dd) - knockout_len));
    d2h = -d2h;
end


% Discovers a single peak by searching computed second derivative dd,
% eliminates the discovered peak from further matching, and computes a range
% over which to fit the peak.
function [full_range, dd, m, len] = ...
    find_one_peak(aiq, fp, dd, smoothing, threshold)

    [m, ix] = min(dd);

    left = ix;
    while left > 1 && fp(left-1) >= fp(left); left = left - 1; end
    while left > 1 && fp(left-1) <= fp(left); left = left - 1; end

    limit = length(fp);
    right = ix;
    while right < limit && fp(right+1) >= fp(right); right = right + 1; end
    while right < limit && fp(right+1) <= fp(right); right = right + 1; end

    dd(left:right) = 0;
    len = right - left + 1;

    left_right = smoothing*(left - 1) + 1 : smoothing*right;
    max_p = max(aiq(left_right));
    full_range = find(aiq(left_right) >= threshold * max_p) + left_right(1) - 1;
end


% Initial peak fit.  First find peak by rough detection of data maximum, and
% then slice out surrounding peak for subsequent fitting.
function [fit, model, e] = fit_one_peak(s, iq, range, refit)
    sr = s(range);
    iqr = iq(range);

    fit = fit_peak(sr, iqr, ones(length(range), 1));

    % Fit the peak twice.  Use the first peak to compute weights to refine
    % results for the second final fit.
    if refit
        fit = fit_peak(sr, iqr, 1 ./ abs(sr - fit(2)).^2);
    end

    model = fit(1) ./ (s - fit(2));

    % Fit error: take mean squared error |(iqr-mr)/mr|^2.
    mr = model(range);
    e = mean((abs(iqr./mr) - 1).^2);
end


% Computes parameters (a, b) for best fit of model z = a/(f-b) weighted by w.
function p = fit_peak(f, z, w)
    z2 = abs(z).^2;
    s = mean(w);
    sz = mean(w.*z);
    sz2 = mean(w.*z2);
    sfz = mean(w.*f.*z);
    sfz2 = mean(w.*f.*z2);

    M = [s sz; conj(sz) sz2];
    V = [sfz; sfz2];
    p = M\V;
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
        weights = weights + 1 ./ abs(s - p(2));
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


function plot_results(s, iq, count, detail, p_out, fig, x_lim)
    function c = colour(n)
        colours = 'brmgcky';
        c = colours(mod(n - 1, length(colours)) + 1);
    end


    figure(fig); clf;

    subplot 221
    hold all
    for n = 1:count
        plot( ...
            s, abs(detail.m(:, n)), ['--' colour(n)], ...
            s, abs(p_out(2*n-1)./(s-p_out(2*n))), colour(n));
    end
    if x_lim; xlim(x_lim); end

    subplot 222
    hold all
    strings = {};
    for n = 1:count
        r = detail.r{n};
        plot(detail.iq(r, n), ['.' colour(n)]);
        plot(detail.m(r, n), colour(n));
        strings{2*n-1} = sprintf('P%d data', n);
        strings{2*n} = sprintf('P%d fit', n);
    end
    legend(strings)
    axis equal

    subplot 223
    semilogy(s, abs(iq), ':', s, abs(detail.m_out))
    if x_lim; xlim(x_lim); end

    subplot 224
    initial_model = sum(detail.m, 2);
    plot(real(iq), imag(iq), ':', ...
        real(initial_model), imag(initial_model), '--', ...
        real(detail.m_out), imag(detail.m_out));
    legend('Raw data', 'Initial fit', 'Final fit')
    axis equal
end

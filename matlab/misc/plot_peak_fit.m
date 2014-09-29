% Reads and plots peak fit data
function [iq, s, first, second] = plot_peak_fit(tmbf)
    [first, second] = read_peak_fit(tmbf);
    iq = [1 1j] * lcaGet({[tmbf ':TUNE:I']; [tmbf ':TUNE:Q']});
    s = lcaGet([tmbf ':DET:SCALE']);
    threshold = lcaGet([tmbf ':PEAK:THRESHOLD_S']);

    m1 = model(s, first.fits(:, first.status == 0));
    m2 = model(s, second.fits(:, second.status == 0));

    clf

    subplot 221
    semilogy(s, abs(iq), s, abs(m1), s, abs(m2));
    xlim([min(s) max(s)])
    legend('IQ', 'First fit', 'Second fit')
    title('Power spectrum')

    subplot 222
    hold all
    plot(make_complex(iq), ':')
    plot(make_complex(m1))
    plot(make_complex(m2))
    axis equal
    legend('IQ', 'First fit', 'Second fit')
    title('Full model fit')

    subplot 223
    plot_result(s, iq, first, [], threshold)
    title('First fits')

    subplot 224
    plot_result(s, iq, second, first.fits(:, 1:first.count), 0)
    title('Second fits')
end

function range = threshold_range(power, range, threshold)
    power = power(range);
    min_iq = max(power) * threshold;
    range = range(find(power >= threshold * max(power)));
end

function plot_result(s, iq, result, first_fit, threshold)
    hold all
    strings = {};
    power = abs(iq).^2;
    for n = 1:result.count
        % r is the range of points over which we've done the fit, th_r is the
        % set of points passing the threshold test.
        r = 1 + (result.ranges(1,n):result.ranges(2,n));
        th_r = threshold_range(power, r, threshold);

        % Plot the thresholded corrected data.  The correction is computed from
        % a combination of the fits successfully completed so far.
        k = [result.fits(:, 1:n-1) first_fit(:, n+1:end)];
        plot(iq(th_r) - model(s(th_r), k), '.')
        strings{2*n-1} = sprintf('P%d data', n);

        % Plot the resulting model over the fitting range
        plot(model(s(r), result.fits(:,n)))
        strings{2*n} = sprintf('P%d fit, e: %.3f', n, result.errors(n));
    end
    plot(complex(0), 'x')
    if ~isempty(strings); legend(strings); end
    axis equal
end

function z = model(s, fit)
    z = zeros(size(s));
    for p = fit
        z = z + p(1) ./ (s - p(2));
    end
end

% This one is a bit annoying: in some versions of matlab it's not valid to call
% complex(z) on a complex argument z.  This is fixed now apparently...
function z = make_complex(z)
    if isreal(z)
        z = complex(z);
    end
end

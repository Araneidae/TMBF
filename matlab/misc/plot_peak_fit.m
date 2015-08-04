% Reads and plots peak fit data
function [iq, s, first, second] = plot_peak_fit(tmbf)
    [first, second] = read_peak_fit(tmbf);
    iq = [1 1j] * lcaGet({[tmbf ':TUNE:I']; [tmbf ':TUNE:Q']});
    s = lcaGet([tmbf ':DET:SCALE']);

    first_fits = first.fits(:, first.status == 0);
    second_fits = second.fits(:, second.status == 0);
    m1 = model(s, first_fits);
    m2 = model(s, second_fits);

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
    plot_result(s, iq, first, [])
    title('First fits')

    subplot 224
    % Need to recompute the fits going into the second fit: sort the fits in
    % descending order of area and extract the largest peaks.
    areas = abs(first_fits(1,:)).^2 ./ -imag(first_fits(2,:));
    [areas, ix] = sort(areas, 2, 'descend');
    first_fits = first_fits(:, ix(1:size(second_fits, 2)));
    plot_result(s, iq, second, first_fits)
    title('Second fits')
end

function plot_result(s, iq, result, first_fit)
    hold all
    strings = {};
    power = abs(iq).^2;
    for n = 1:result.count
        % r is the range of points over which we've done the fit
        r = 1 + (result.ranges(1,n):result.ranges(2,n));

        % Plot the thresholded corrected data.  The correction is computed from
        % a combination of the fits successfully completed so far.
        k = [result.fits(:, 1:n-1) first_fit(:, n+1:end)];
        plot(iq(r) - model(s(r), k), '.')
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

% Reads and plots peak fit data
function [iq, s, first, second] = plot_peak_fit(tmbf)
    [first, second] = read_peak_fit(tmbf);
    iq = [1 1j] * lcaGet({[tmbf ':TUNE:I']; [tmbf ':TUNE:Q']});
    s = lcaGet([tmbf ':DET:SCALE']);

    m1 = model(s, first.fits(:, first.status == 0));
    m2 = model(s, second.fits(:, second.status == 0));

    clf

    subplot 221
    semilogy(s, abs(iq), s, abs(m1), s, abs(m2));
    xlim([s(1) s(end)]);
    legend('IQ', 'First fit', 'Second fit')
    title('Power spectrum')

    subplot 222
    hold all
    plot(iq, ':')
    plot(m1)
    plot(m2)
    axis equal
    legend('IQ', 'First fit', 'Second fit')
    title('Full model fit')

    subplot 223
    plot_result(s, iq, first, true)
    title('First fits')

    subplot 224
    plot_result(s, iq, second, false)
    title('Second fits')
end

function plot_result(s, iq, result, first_fit)
    hold all
    strings = {};
    for n = 1:result.count
        % Compute correction to IQ data so that we show the data we really
        % fitted against.
        if first_fit
            k = result.fits(:, 1:n-1);
        else
            k = result.fits(:, [1:n-1 n+1:result.count]);
        end

        r = 1 + (result.ranges(1,n):result.ranges(2,n));
        plot(iq(r) - model(s(r), k), '.')
        plot(model(s(r), result.fits(:,n)))

        strings{2*n-1} = sprintf('P%d data', n);
        strings{2*n} = sprintf('P%d fit', n);
    end
    legend(strings)
    axis equal
end

function z = model(s, fit)
    z = zeros(size(s));
    for p = fit
        z = z + p(1) ./ (s - p(2));
    end
end

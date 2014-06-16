% Wrapper for testing fit_fir
function test_fit(f0, delta, delay, phi0, N, alpha, beta)
    subplot(1,1,1); plot([0 1]);

    if ~exist('alpha', 'var')
        alpha = logspace(-5, -1, 5);
    end

    range_all = [0 0.5];
    range_sub = f0 + [-delta delta];
    for a = alpha
        [f, ff] = fit_fir(2*pi*f0, 2*pi*delta, delay, 2*pi*phi0, N, a, beta);
        disp(f');

        plotit([1 2], range_all, abs(ff), 'Magnitude');
        plotit([3 4], range_sub, abs(ff), 'Magnitude'); ylim([0.8 1.2]);

        ang = unwrap(angle(ff))/2/pi;
        plotit([6 7], range_all, ang, 'Angle');
        plotit([8 9], range_sub, ang, 'Angle');

%        group_delay = diff(ang) * 1024;
        phase_error = ang - delay * (linspace(0, 1, length(ff))' - f0) - phi0;
        plotit([11 12], range_all, phase_error, 'Phase error');
        plotit([13 14], range_sub, phase_error, 'Phase error');
        ylim([-0.1 0.1]);

        subplot(2,5,[5 10]);
        hold all;
        stem(f);
        title('Filter');
    end
end

function plotit(sub, range, ff, title_text)
    subplot(3,5,sub);
    hold all
    plot(linspace(0, 1, length(ff)), ff);
    xlim(range);
    title(title_text);
end

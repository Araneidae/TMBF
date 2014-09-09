% Runs fit_peaks over a data set

function [pp, nn, ee, dd] = run_fit_peaks(s, iq, peak_count, varargin)
    if ~exist('peak_count', 'var'); peak_count = 3; end

    wf_len = size(iq, 1);
    try_count = size(iq, 2);

    pp = zeros(try_count, 2 * peak_count);
    nn = zeros(try_count, peak_count);
    ee = zeros(try_count, peak_count);
    dd = zeros(try_count, peak_count);
    bg = zeros(try_count, 1);

    for n = 1:try_count
        [p, r] = fit_peaks(s, iq(:, n), 'count', peak_count, varargin{:});
        pp(n, :) = p;
        ee(n, :) = r.e;
        dd(n, :) = r.d2h;
        nn(n, :) = r.n;
        bg(n) = r.bg;
    end

    alpha = abs(pp(:, 1:2:end));
    phase = 180/pi * angle(pp(:, 1:2:end));
    width = -imag(pp(:, 2:2:end));
    s0 = real(pp(:, 2:2:end));

    height = alpha ./ width;
    area = alpha .* height;

    s_bin_width = diff(s([1 end]))/wf_len;

    figure(2); clf

    subplot 231
    plot_semilogy(height, '.')
    title('Peak height')

    subplot 232
    plot_semilogy(width, '.')
    hold all
    semilogy([1 try_count], s_bin_width * [1 1], 'r--')
    title('Peak width')

    subplot 233
    plot_semilogy(area, '.')
    title('Peak area')

    subplot 234
    plot_semilogy(alpha, '.')
    title('Peak alpha')

    subplot 235
    plot_semilogy(width./(nn * s_bin_width), '.')
    title('Fitted width ratio')

    subplot 236
    plot_semilogy(dd, '.')
    hold all
    semilogy(bg, 'r')
    title('Second derivative peak height')
end

function plot_semilogy(data, varargin)
    least = min(min(data(find(data > 0))));
    data(find(data <= 0)) = least / 10;
    semilogy(data, varargin{:});
end

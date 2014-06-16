% Analyses Tune PLL debug data.  Unfortunately is now out of step with current
% implementation, so produces unhelpful results.

k = 2 / sqrt(prod(1 + 2.^-(2*(0:15))));

if ~exist('worst_e'); worst_e = 0; end
while 1;
    d = [1 1j 0 0; 0 0 1 0; 0 0 0 1] * ...
        reshape(lcaGet('TS-DI-TMBF-01:BUF:WF'), 4, []);
    iq = d(1,:);
    a = d(2,:) / 2^18*2*pi;
    m = k * d(3,:);

    niq = m .* exp(1j*a);

    clf;
    subplot(1, 3, 1:2);
    plot(iq,'.');
    axis equal;
    title('Captured IQ data');

    e = niq - iq;
    try
        [cen, rad] = minboundcircle(real(e), imag(e));
        cen = [1 1j] * cen';
    catch
        cen = mean(e);
        rad = 0;
    end
    c = cen + rad * exp(1j * linspace(0, 2*pi));

    % Capture worst error point
    [w, ix] = max(abs(e));
    if w > worst_e
        worst_e = w;
        worst_iq = iq(ix);
    end

    subplot(2, 3, 3);
    plot(real(e), imag(e), '.', real(c), imag(c), '--');
%    axis([-5 5 -5 5]);
    axis equal;
    title(sprintf('Error, var = %g, rad = %g', var(e), rad));

    subplot(4, 3, 9);
    plot(a);
    title('Angle');
    subplot(4, 3, 12);
    plot(m);
    title('Magnitude');

    drawnow;
    pause(0.2);
end

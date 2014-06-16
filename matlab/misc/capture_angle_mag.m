% With TUNE PLL following on sweeps +-45 degrees around given centre frequency
% and plots angle and magnitude.
function [ang, mag, freq] = capture_angle_mag(tmbf, centre, dwell)
    target_pv = [tmbf ':FTUN:TARGET_S'];
    mag_pv = [tmbf ':FTUN:MAG'];
    freq_pv = [tmbf ':NCO:FREQ'];

    ang = centre-45:centre+45;
    N = length(ang);
    mag = zeros(1, N);
    freq = zeros(1, N);

    for n=1:N
        lcaPut(target_pv, ang(n));
        pause(dwell);
        mag(n) = lcaGet(mag_pv);
        freq(n) = lcaGet(freq_pv);
    end
    lcaPut(target_pv, centre);

    subplot(2,2,1)
    plot(ang, mag)
    subplot(2,2,3)
    plot(ang, freq)
    subplot(2,2,[2 4])
    plot(mag .* exp(2j*pi * ang/360))
    axis equal
end

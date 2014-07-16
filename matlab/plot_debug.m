% Function to plot debug data.
% Must be invoked with tmbf set to name of target device.

[iq, mag, raw_angle, filtered_angle, feedback_freq, status] = ...
    tmbf_read_debug(tmbf);

N = length(iq);

subplot(3, 3, [1 2])
plot(1:N, real(iq), 1:N, imag(iq))
xlim([1 N])

subplot(3, 3, 3)
plot(iq, '.')
axis equal

subplot(3, 2, 3)
plot(1:N, raw_angle, 1:N, filtered_angle, 'r')
xlim([1 N])

subplot(3, 2, 4)
plot(mag)
xlim([1 N])

subplot(3, 2, 5)
plot(feedback_freq)
xlim([1 N])

subplot(3, 2, 6)
plot(0.8 * status + repmat(0:11, N, 1))
axis([1 N -0.2 12])

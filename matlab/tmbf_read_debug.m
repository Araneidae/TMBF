% [iq, mag, raw_angle, filtered_angle, feedback_freq, status ...
%     ] = tmbf_read_debug(varargin)
%
% Reads tune following debug data from TMBF buffer, which must be in Debug mode.

function [iq, mag, raw_angle, filtered_angle, feedback_freq, status ...
    ] = tmbf_read_debug(varargin)

    data = tmbf_read_raw('Debug', varargin{:});

    iq = squeeze(data(1, 1, :) + 1j * data(1, 2, :));
    mag = squeeze(data(2, 1, :));
    raw_angle = 360 * 2^-16 * squeeze(data(2, 2, :));

    row3 = typecast(int16(reshape(data(3, :, :), [], 1)), 'int32');
    filtered_angle = 360 * 2^-18 * double(row3);

    row4 = typecast(int16(reshape(data(4, :, :), [], 1)), 'uint32');
    feedback_freq = 936 * 2^-32 * 2^-14 * double( ...
        typecast(bitshift(row4, 14), 'int32'));

    raw_status = bitshift(row4, -18);
    status = logical(zeros(length(raw_status), 12));
    for n = 1:12
        status(:, n) = bitget(raw_status, n);
    end
end

% Reads and decodes peak fit data
function [first, second] = read_peak_fit(tmbf)
    first  = decode_fit(lcaGet([tmbf ':PEAK:FIRSTFIT']));
    second = decode_fit(lcaGet([tmbf ':PEAK:SECONDFIT']));
end

function fit = decode_fit(data)
    data = int8(data);

    % Decoding structure fields, this code must match the structure definition
    % for peak_fit_result in tune_peaks.c
    %
    %   start   field                   size
    %   -----   -----                   ----
    %   0       peak_count              4
    %   4       ranges[MAX_PEAKS]       3 * (4 + 4) = 24
    %   28      (padding)               4
    %   32      fits[MAX_PEAKS]         3 * (16 + 16) = 96
    %   128     errors[MAX_PEAKS]       3 * 8 = 24
    %   152     status[MAX_PEAKS]       3 * 4 = 12
    %   164     (padding)               4
    %
    % One needs to be added to all these fields to create matlab offsets.

    fit = {};
    fit.count = typecast(data(1:4), 'int32');
    fit.ranges = reshape(typecast(data(5:28), 'int32'), 2, 3);
    fits = typecast(data(33:128), 'double');
    fits = [1 1j] * reshape(fits, 2, 6);
    fit.fits = reshape(fits, 2, 3);
    fit.errors = typecast(data(129:152), 'double');
    fit.status = typecast(data(153:164), 'int32');
end

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
    % The header of this structure contains offsets of the remaining fields.
    max_peaks     = read_int32(data, 0);
    count_offset  = read_int32(data, 4);
    ranges_offset = read_int32(data, 8);
    fits_offset   = read_int32(data, 12);
    errors_offset = read_int32(data, 16);
    status_offset = read_int32(data, 20);

    fit = {};
    fit.count = read_int32(data, count_offset);
    fit.ranges = reshape( ...
        read_int32_array(data, ranges_offset, 2*max_peaks), 2, []);
    fit.fits = reshape( ...
        read_complex_array(data, fits_offset, 2*max_peaks), 2, []);
    fit.errors = read_double_array(data, errors_offset, max_peaks);
    fit.status = read_int32_array(data, status_offset, max_peaks);
end

function result = read_int32(data, offset)
    result = typecast(data(offset+1:offset+4), 'int32');
end

function result = read_int32_array(data, offset, length)
    result = typecast(data(offset+1:offset+4*length), 'int32');
end

function result = read_double_array(data, offset, length)
    result = typecast(data(offset+1:offset+8*length), 'double');
end

function result = read_complex_array(data, offset, length)
    result = [1 1i] * reshape( ...
        read_double_array(data, offset, 2*length), 2, []);
end

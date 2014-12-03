% data = tmbf_read_raw(expect, tmbf, count)
%
% Reads raw number of samples from DDR buffer, returns as 4 x 2 x count
% waveform.  The expect argument should match the input source of the buffer.

function [data, bunches] = tmbf_read_raw(expect, tmbf, count)
    % Check DDR is in the expected capture mode
    input = lcaGet([tmbf ':DDR:INPUT_S']);
    assert(strcmp(input, expect), ...
        sprintf('Expect %s:DDR to be capturing %s, not %s', ...
        tmbf, expect, input{:}));

    if ~exist('count', 'var')
        count = lcaGet([tmbf ':DDR:COUNT']);
    end

    bunches = lcaGet([tmbf ':BUNCHES']);
    data = tmbf_read(tmbf, ceil(8 * count / bunches));
    data = reshape(data, 4, 2, []);
    data = data(:, :, 1:count);
end

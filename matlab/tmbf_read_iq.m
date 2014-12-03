% iq = tmbf_read_iq(tmbf [, count [, mode]])
%
% Reads IQ data from DDR buffer which must be in IQ capture mode.

function iq = tmbf_read_iq(tmbf, count, mode)
    % Check DDR is in the expected capture mode
    input = lcaGet([tmbf ':DDR:INPUT_S']);
    assert(strcmp(input, 'IQ'), ...
        sprintf('Expect %s:DDR to be capturing IQ, not %s', tmbf, input{:}));

    % Set the IQ readout mode if requested and read it back for a definitive
    % reading.
    mode_pv = [tmbf ':DDR:IQMODE_S'];
    if exist('mode', 'var')
        lcaPut(mode_pv, mode)
    end
    mode = lcaGet(mode_pv);

    if ~exist('count', 'var')
        count = lcaGet([tmbf ':DDR:COUNT']);
    end

    bunches = lcaGet([tmbf ':BUNCHES']);
    if strcmp(mode, 'All')
        data = tmbf_read(tmbf, ceil(8 * count / bunches));
        data = reshape(data, 4, 2, []);
        data = data(:, :, 1:count);
        iq = squeeze(data(:, 1, :) + 1j * data(:, 2, :)).';
    else
        data = tmbf_read(tmbf, ceil(2 * count / bunches));
        data = reshape(data, 2, []);
        data = data(:, 1:count);
        iq = [1 1j] * data;
    end
end

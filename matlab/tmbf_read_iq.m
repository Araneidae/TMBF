% iq = tmbf_read_iq(tmbf [, count])
%
% Reads IQ data from DDR buffer which must be in IQ capture mode.

function iq = tmbf_read_iq(varargin)
    data = tmbf_read_raw('IQ', varargin{:});
    iq = squeeze(data(:, 1, :) + 1j * data(:, 2, :)).';
end

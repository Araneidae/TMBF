% iq = tmbf_read_iq(tmbf [, count])

function iq = tmbf_read_iq(varargin)
    data = tmbf_read_raw('IQ', varargin{:});
    iq = squeeze(data(:, 1, :) + 1j * data(:, 2, :)).';
end

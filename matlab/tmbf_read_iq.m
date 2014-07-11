% iq = tmbf_read_iq(tmbf, count)

function iq = tmbf_read_iq(tmbf, count)
    if ~exist('count', 'var')
        count = lcaGet([tmbf ':DDR:COUNT']);
    end
    x = tmbf_read(tmbf, ceil(8 * count / 936));
    iq = reshape([1 1j] * ...
        reshape(permute(reshape(x, 4, 2, []), [2 1 3]), 2, []), 4, []).';
    count = min(count, size(iq, 1));
    iq = iq(1:count, :);
end

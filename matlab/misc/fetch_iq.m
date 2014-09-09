% Retrieve IQ data from archiver
function [s, wfiq, t] = fetch_iq(xy, start)
    tmbf = sprintf('SR23C-DI-TMBF-%02d', xy);
    s = ar([tmbf ':DET:SCALE'], start, 1);
    [wfi, ti] = ar([tmbf ':TUNE:I'], start, 250);
    [wfq, tq] = ar([tmbf ':TUNE:Q'], start, 250);
    assert(isempty(find(ti - tq)));
    t = ti;
    wfiq = wfi + 1j * wfq;

    imagesc(t, s, abs(wfiq))
    datetick
    title(sprintf('%s: %s', tmbf, datestr(t(1))))
end

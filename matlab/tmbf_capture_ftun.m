% Captures given number of points of Tune PLL data.  Will hang if not running.
function ftun = tmbf_capture_ftun(tmbf, count)
    pv = [tmbf ':FTUN:FREQ'];
    lcaSetMonitor(pv);

    wh = waitbar(0, 'Fetching data', ...
        'CreateCancelBtn', 'setappdata(gcbf,''cancelling'',1)');
    onCleanup(@() delete(wh));
    setappdata(wh, 'cancelling', 0);

    ftun = [];
    while length(ftun) < count
        while ~lcaNewMonitorValue(pv); pause(0.1); end
        ftun = [ftun lcaGet(pv)];

        waitbar(length(ftun) / count, wh);
        if getappdata(wh, 'cancelling')
            break
        end
    end
    lcaClear(pv);
end

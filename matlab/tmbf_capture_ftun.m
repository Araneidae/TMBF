%   ftun = tmbf_capture_ftun(tmbf, count, [progress_bar])
% Captures given number of points of Tune PLL data.  Will hang if not running.
function ftun = tmbf_capture_ftun(tmbf, count, varargin)
    pv = [tmbf ':FTUN:FREQ'];
    lcaSetMonitor(pv);

    bar = progress_bar('Fetching data', varargin{:});

    ftun = [];
    while length(ftun) < count
        while ~lcaNewMonitorValue(pv); pause(0.1); end
        ftun = [ftun lcaGet(pv)];
        if ~bar.advance(length(ftun) / count); break; end
    end
    lcaClear(pv);
end

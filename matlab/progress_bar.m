% bar = progress_bar(title)
%
% Creates a progress bar which can be updated periodically to show data capture
% or computation progress.  The progress bar will be removed when the returned
% variable bar is deleted.
%
% Call the following functions on the progress bar to check and update it:
%
%   ok = bar.advance(fraction)
%       Update progress bar to show progress as fraction done.  The fraction
%       should be a number in the range 0 to 1.  Returns false if the cancel
%       button has been pressed.
%
%   ok = bar.cancelled()
%       Returns false if the cancel bar has been pressed, does not update bar.
function bar = progress_bar(title)
    function ok = advance_waitbar(bar, fraction)
        waitbar(fraction, bar.wb);
        ok = ~getappdata(bar.wb, 'cancelling');
    end

    function ok = show_advance(fraction)
        fprintf(2, '%5.2f%%\r', 100 * fraction)
        ok = true;
    end

    bar = {};
    if usejava('desktop')
        bar.wb = waitbar(0, title, ...
            'CreateCancelBtn', 'setappdata(gcbf,''cancelling'',1)');
        bar.cleanup = onCleanup(@() delete(bar.wb));
        setappdata(bar.wb, 'cancelling', 0);

        bar.advance = @(fraction) advance_waitbar(bar, fraction);
        bar.cancelled = @() bar.cancelling;
    else
        bar.cleanup = onCleanup(@() fprintf(2, 'Done   \n'));
        bar.advance = @show_advance;
        bar.cancelled = @() false;
    end
end

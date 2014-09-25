% data = tmbf_read(tmbf, turns, start)
%
% Reads the selected number of turns from the given tmbf, optionally starting at
% the given offset into the buffer.  The buffer should already be triggered,
% call tmbf_trigger first if necessary.
%
% turns defaults to all available turns if not specified, start defaults to the
% trigger point.  A starting offset of up to +-35846 turns can be specified
% and the length can be up to 2*35846 depending on the starting offset.
function data = tmbf_read(tmbf, turns, start)
    max_turns = 35846;
    if nargin < 2; turns = 8962; end
    if nargin < 3; start = 0; end
    assert(-max_turns <= start  &&  start < max_turns, 'Invalid start');
    assert(start + turns <= max_turns, 'Too many turns requested');
    assert(turns > 0, 'Bad number of turns requested');

    status   = [tmbf ':DDR:STATUS'];
    set_turn = [tmbf ':DDR:TURNSEL_S'];
    longwf   = [tmbf ':DDR:LONGWF'];

    % Ensure DDR capture isn't running before trying to read
    assert(strcmp(lcaGet(status), 'Ready'), [tmbf ' is not ready']);

    turn_length = 936;
    window_length = lcaGet([longwf '.NELM']) / turn_length;

    bar = progress_bar('Fetching data');

    data = zeros(turns * turn_length, 1);
    turns_read = 0;
    while turns_read < turns;
        % Move readout window to appropriate position.  We rely on lcaPut
        % blocking until the window has updated.
        turns_start = turns_read + start;
        lcaPut(set_turn, turns_start);
        start_time = now;

        % Read selected window into buffer
        bunches_size  = turn_length * min(window_length, turns - turns_read);
        bunches_start = turn_length * turns_read;
        bunches_end = bunches_start + bunches_size;
        wf = lcaGet(longwf);
        data(bunches_start + 1 : bunches_end) = wf(1 : bunches_size);

        turns_read = turns_read + window_length;
        if ~bar.advance(turns_read / turns)
            data = data(1:bunches_end);
            break
        end
    end
end

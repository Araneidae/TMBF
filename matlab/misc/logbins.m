% Logarithmic binning.
function [y_out, x_out] = logbins(x_in, y_in, bins)
    if ~exist('bins', 'var')
        bins = logspace(log10(x_in(2)), log10(x_in(end)));
    end

    % Take geometric means of bin boundaries for x_out
    x_out = sqrt(0.5 * (bins(1:end-1).^2 + bins(2:end).^2));

    % Bin the input data.  Don't know a way to do this in vectorised style
    bin_widths = diff(bins);
    y_out = zeros(size(bin_widths));
    keep = zeros(size(bin_widths));
    for n = 1:length(bins)-1
        fbins = find(bins(n) <= x_in & x_in < bins(n+1));
        if fbins
            y_out(n) = mean(y_in(fbins));
            keep(n) = 1;
        end
    end

    % Strip out empty bins
    x_out = x_out(keep ~= 0);
    y_out = y_out(keep ~= 0);
end

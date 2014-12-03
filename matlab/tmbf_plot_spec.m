function [ss, x] = tmbf_plot_spec(tmbf, n, lines)
    lcaPut([tmbf ':TRG:DDR:RESET_S'], 0);
    lcaPut([tmbf ':DDR:INPUT_S'], 'ADC');
    lcaPut([tmbf ':TRG:DDR:SEL_S'], 'Soft');
    lcaPut([tmbf ':TRG:DDR:MODE_S'], 'One Shot');
    lcaPut([tmbf ':TRG:DDR:ARM_S.PROC'], 1);
    pause(2)
    [x, bunches] = tmbf_read(tmbf, n);

    l=length(x);
    % an alternative to the revolution line removal below. the effect is the
    % same

    xx=reshape(x, bunches, []); %turn into matrix bunches x turns
    xx = xx-repmat(mean(xx, 2), 1, n); %subtract the average position per bunch
    x=reshape(xx, 1, []); %strech out again

    s=2*abs(fft(x))/l;%calculate spectrum over all bunches
    ss=reshape(s, n, bunches);%fold into tune x modes
    figure(1)
    clf
    if ~exist('lines', 'var')
        subplot('position', [.05 .35 .7 .6])
    end
    imagesc([1 bunches], [0 .5], log10(ss(1:end/2, :)), [-2 log10(max(s))]);
    set(gca, 'YDir', 'normal')
    if exist('lines', 'var')
        xlabel('modes')
        ylabel('tune')
    else
        s2=sum(ss(1:end/2, :).^2, 2);
        [p, pi]=max(s2);
        f=linspace(0, .5, length(s2));
        subplot('position', [.75 .35 .2 .6])
        plot(s2, f);
        set(gca, 'YAxisLocation', 'right');
        set(gca, 'XAxisLocation', 'top')

        subplot('position', [.05 .1 .7 .25])
        plot(ss(pi, :))
        xlabel(sprintf('Modes at peak tune %3.3f', f(pi)))
        axis tight
    end
    figure(2)
    clf
    xf=abs(fft(xx, [], 2))/n;
    % only taking the lower half of the FFT
    xf=xf(:, 1:end/2).';
    s2=sum(xf.^2, 2);

    if ~exist('lines', 'var')
        subplot('position', [.05 .35 .7 .6])
    end
    imagesc([1 bunches], [0 .5], log10(xf), [-1 log10(max(max(xf)))]);
    set(gca, 'YDir', 'normal')

    if exist('lines', 'var')
        xlabel('modes')
        ylabel('tune')
    else
        subplot('position', [.75 .35 .2 .6])
        plot(s2, linspace(0, .5, length(s2)));
        xlabel('Normalised Signal');
        set(gca, 'YAxisLocation', 'right');
        set(gca, 'XAxisLocation', 'top')

        subplot('position', [.05 .1 .7 .25])
        plot(xf(pi, :))
        xlabel(sprintf('Bunches at peak tune %3.3f', f(pi)))
        axis tight
    end
end

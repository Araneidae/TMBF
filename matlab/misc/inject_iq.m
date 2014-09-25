function inject_iq(tmbf, iq, s)
    iq = iq(:).';
    lcaPut([tmbf ':TUNE:INJECT:I_S'], real(iq));
    lcaPut([tmbf ':TUNE:INJECT:Q_S'], imag(iq));
    if exist('s', 'var')
        lcaPut([tmbf ':TUNE:INJECT:S_S'], s);
    end
    lcaPut([tmbf ':TUNE:INJECT_S'], 0);
end

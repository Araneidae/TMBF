% Automatically computes and applies ADC offsets to cancel out ADC variation.
function offsets = tmbf_adc_offset(tmbf)
    offsets_pv = [tmbf ':ADC:OFFSET_S'];

    tmbf_trigger(tmbf);
    wf = tmbf_read(tmbf, 1024);
    offsets = lcaGet(offsets_pv) - round(mean(reshape(wf, 4, [])'));
    lcaPut(offsets_pv, offsets);
end

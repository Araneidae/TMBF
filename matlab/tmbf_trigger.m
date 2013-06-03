% Ensures TMBF is in single shot mode and waits for trigger
function tmbf_trigger(tmbf)
    trigmode = [tmbf ':DDR:TRIGMODE_S'];
    arm      = [tmbf ':DDR:ARM_S.PROC'];
    ready    = [tmbf ':DDR:READY'];

    lcaPut(trigmode, 'One Shot');
    lcaPut(arm, 0);
    while ~strcmp(lcaGet(ready), 'Triggered'); pause(0.01); end
end

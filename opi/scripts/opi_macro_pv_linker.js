importPackage(Packages.org.csstudio.opibuilder.scriptUtil);
importPackage(Packages.org.csstudio.opibuilder.util);

macroUtil = OPIBuilderMacroUtil();
macroMap = macroUtil.getWidgetMacroMap(widget.getWidgetModel());

for (var i = 0; i < pvs.length; i++) {
    key = pvs[i].toString().replaceAll('^loc://DID_[0-9]*', '');
    ConsoleUtil.writeInfo(key + ": " + PVUtil.getDouble(pvs[i]));
    macroMap.put(key, PVUtil.getDouble(pvs[i]).toString());
}

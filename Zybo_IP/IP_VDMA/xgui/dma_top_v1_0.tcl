# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "FRAME_HEIGHT" -parent ${Page_0}
  ipgui::add_param $IPINST -name "MM2S_FIFO_DEPTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "PIXELS_PER_LINE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "S2MM_FIFO_DEPTH" -parent ${Page_0}


}

proc update_PARAM_VALUE.FRAME_HEIGHT { PARAM_VALUE.FRAME_HEIGHT } {
	# Procedure called to update FRAME_HEIGHT when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.FRAME_HEIGHT { PARAM_VALUE.FRAME_HEIGHT } {
	# Procedure called to validate FRAME_HEIGHT
	return true
}

proc update_PARAM_VALUE.MM2S_FIFO_DEPTH { PARAM_VALUE.MM2S_FIFO_DEPTH } {
	# Procedure called to update MM2S_FIFO_DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.MM2S_FIFO_DEPTH { PARAM_VALUE.MM2S_FIFO_DEPTH } {
	# Procedure called to validate MM2S_FIFO_DEPTH
	return true
}

proc update_PARAM_VALUE.PIXELS_PER_LINE { PARAM_VALUE.PIXELS_PER_LINE } {
	# Procedure called to update PIXELS_PER_LINE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.PIXELS_PER_LINE { PARAM_VALUE.PIXELS_PER_LINE } {
	# Procedure called to validate PIXELS_PER_LINE
	return true
}

proc update_PARAM_VALUE.S2MM_FIFO_DEPTH { PARAM_VALUE.S2MM_FIFO_DEPTH } {
	# Procedure called to update S2MM_FIFO_DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.S2MM_FIFO_DEPTH { PARAM_VALUE.S2MM_FIFO_DEPTH } {
	# Procedure called to validate S2MM_FIFO_DEPTH
	return true
}


proc update_MODELPARAM_VALUE.PIXELS_PER_LINE { MODELPARAM_VALUE.PIXELS_PER_LINE PARAM_VALUE.PIXELS_PER_LINE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PIXELS_PER_LINE}] ${MODELPARAM_VALUE.PIXELS_PER_LINE}
}

proc update_MODELPARAM_VALUE.FRAME_HEIGHT { MODELPARAM_VALUE.FRAME_HEIGHT PARAM_VALUE.FRAME_HEIGHT } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.FRAME_HEIGHT}] ${MODELPARAM_VALUE.FRAME_HEIGHT}
}

proc update_MODELPARAM_VALUE.S2MM_FIFO_DEPTH { MODELPARAM_VALUE.S2MM_FIFO_DEPTH PARAM_VALUE.S2MM_FIFO_DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.S2MM_FIFO_DEPTH}] ${MODELPARAM_VALUE.S2MM_FIFO_DEPTH}
}

proc update_MODELPARAM_VALUE.MM2S_FIFO_DEPTH { MODELPARAM_VALUE.MM2S_FIFO_DEPTH PARAM_VALUE.MM2S_FIFO_DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.MM2S_FIFO_DEPTH}] ${MODELPARAM_VALUE.MM2S_FIFO_DEPTH}
}


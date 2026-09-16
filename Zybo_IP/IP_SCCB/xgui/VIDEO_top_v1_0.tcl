# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "BYTE_SWAP" -parent ${Page_0}
  ipgui::add_param $IPINST -name "cam_x" -parent ${Page_0}
  ipgui::add_param $IPINST -name "cam_y" -parent ${Page_0}
  ipgui::add_param $IPINST -name "vga_x" -parent ${Page_0}
  ipgui::add_param $IPINST -name "vga_y" -parent ${Page_0}


}

proc update_PARAM_VALUE.BYTE_SWAP { PARAM_VALUE.BYTE_SWAP } {
	# Procedure called to update BYTE_SWAP when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BYTE_SWAP { PARAM_VALUE.BYTE_SWAP } {
	# Procedure called to validate BYTE_SWAP
	return true
}

proc update_PARAM_VALUE.cam_x { PARAM_VALUE.cam_x } {
	# Procedure called to update cam_x when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.cam_x { PARAM_VALUE.cam_x } {
	# Procedure called to validate cam_x
	return true
}

proc update_PARAM_VALUE.cam_y { PARAM_VALUE.cam_y } {
	# Procedure called to update cam_y when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.cam_y { PARAM_VALUE.cam_y } {
	# Procedure called to validate cam_y
	return true
}

proc update_PARAM_VALUE.vga_x { PARAM_VALUE.vga_x } {
	# Procedure called to update vga_x when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.vga_x { PARAM_VALUE.vga_x } {
	# Procedure called to validate vga_x
	return true
}

proc update_PARAM_VALUE.vga_y { PARAM_VALUE.vga_y } {
	# Procedure called to update vga_y when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.vga_y { PARAM_VALUE.vga_y } {
	# Procedure called to validate vga_y
	return true
}


proc update_MODELPARAM_VALUE.cam_x { MODELPARAM_VALUE.cam_x PARAM_VALUE.cam_x } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.cam_x}] ${MODELPARAM_VALUE.cam_x}
}

proc update_MODELPARAM_VALUE.cam_y { MODELPARAM_VALUE.cam_y PARAM_VALUE.cam_y } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.cam_y}] ${MODELPARAM_VALUE.cam_y}
}

proc update_MODELPARAM_VALUE.vga_x { MODELPARAM_VALUE.vga_x PARAM_VALUE.vga_x } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.vga_x}] ${MODELPARAM_VALUE.vga_x}
}

proc update_MODELPARAM_VALUE.vga_y { MODELPARAM_VALUE.vga_y PARAM_VALUE.vga_y } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.vga_y}] ${MODELPARAM_VALUE.vga_y}
}

proc update_MODELPARAM_VALUE.BYTE_SWAP { MODELPARAM_VALUE.BYTE_SWAP PARAM_VALUE.BYTE_SWAP } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BYTE_SWAP}] ${MODELPARAM_VALUE.BYTE_SWAP}
}


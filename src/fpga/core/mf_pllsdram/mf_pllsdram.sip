set_global_assignment -entity "mf_pllsdram" -library "lib_mf_pllsdram" -name IP_TOOL_NAME "altera_pll"
set_global_assignment -entity "mf_pllsdram" -library "lib_mf_pllsdram" -name IP_TOOL_VERSION "23.1"
set_global_assignment -entity "mf_pllsdram" -library "lib_mf_pllsdram" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_mf_pllsdram" -name SPD_FILE [file join $::quartus(sip_path) "mf_pllsdram.spd"]

set_global_assignment -library "lib_mf_pllsdram" -name MISC_FILE [file join $::quartus(sip_path) "mf_pllsdram_sim/mf_pllsdram.vo"]

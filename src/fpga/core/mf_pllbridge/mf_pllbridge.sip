set_global_assignment -entity "mf_pllbridge" -library "lib_mf_pllbridge" -name IP_TOOL_NAME "altera_pll"
set_global_assignment -entity "mf_pllbridge" -library "lib_mf_pllbridge" -name IP_TOOL_VERSION "23.1"
set_global_assignment -entity "mf_pllbridge" -library "lib_mf_pllbridge" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_mf_pllbridge" -name SPD_FILE [file join $::quartus(sip_path) "mf_pllbridge.spd"]

set_global_assignment -library "lib_mf_pllbridge" -name MISC_FILE [file join $::quartus(sip_path) "mf_pllbridge_sim/mf_pllbridge.vo"]

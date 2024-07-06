# data access delay (tAC)
set_input_delay -clock {ic|mp2|mf_pllsdram_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk} -max 6.4 [get_ports {dram_dq[*]}]

# data output hold time (tOH)
set_input_delay -clock {ic|mp2|mf_pllsdram_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk} -min 3.1 [get_ports {dram_dq[*]}]

# data input setup time (tIS)
set_output_delay -clock {ic|mp2|mf_pllsdram_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk} -max 1.5 [get_ports {dram_a[*] dram_ba[*] dram_dq[*] dram_ras_n dram_cas_n dram_we_n }]

# data input hold time (tIH)
set_output_delay -clock {ic|mp2|mf_pllsdram_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk} -min -0.8 [get_ports {dram_a[*] dram_ba[*] dram_dq[*] dram_ras_n dram_cas_n dram_we_n }]

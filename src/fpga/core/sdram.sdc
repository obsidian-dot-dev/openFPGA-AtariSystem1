create_generated_clock -name dram_clk \
  -source [get_pins -compatibility_mode {ic|mp1|mf_pllbase_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] \
  [get_ports {dram_clk}]

#set_clock_groups -exclusive -group [get_clocks { dram_clk }]

#set_clock_groups -exclusive -group [get_clocks {emu|pll|pll_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}]

#set_false_path \
#-from {emu|pll|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk} \
#-to   {emu|pll|pll_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}

#set_false_path \
#-from {emu|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk} \
#-to   {emu|pll|pll_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}

# data access delay (tAC)
set_input_delay -clock dram_clk -max 6.0 [get_ports {dram_dq[*]}]

# data output hold time (tOH)
set_input_delay -clock dram_clk -min 2.5 [get_ports {dram_dq[*]}]

# data input setup time (tIS)
set_output_delay -clock dram_clk -max 1.5 [get_ports {dram_a[*] dram_ba[*] dram_dq* dram_ras_n dram_cas_n dram_we_n }]

# data input hold time (tIH)
set_output_delay -clock dram_clk -min -0.8 [get_ports {dram_a[*] dram_ba[*] dram_dq* dram_raw_n dram_cas_n dram_we_n dram_cke}]

# use proper edges for the timing calculations
set_multicycle_path -setup -end \
  -rise_from [get_clocks {dram_clk}] \
  -rise_to   [get_clocks {ic|mp1|mf_pllbase_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}] 2

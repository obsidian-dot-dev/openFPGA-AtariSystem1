#
# user core constraints
#
# put your clock groups in here as well as any net assignments
#
derive_clock_uncertainty

set_clock_groups -asynchronous \
 -group { bridge_spiclk } \
 -group { clk_74a } \
 -group { clk_74b } \
 -group { ic|mp0|mf_pllbridge_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp2|mf_pllsdram_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk } \
 #-group { ic|mp2|mf_pllsdram_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk } \
          
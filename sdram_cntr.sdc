set_time_format -unit ns
create_clock -name {iclk} -period 50MHz [get_ports {i_clk}]
derive_pll_clocks

set SDRAM_CLK PLL1|altpll_component|auto_generated|pll1|clk[1]

create_generated_clock -name pSDRAM_CLK -source $SDRAM_CLK [get_ports { o_sdram_clk }]
set_clock_groups -exclusive -group [list iclk $SDRAM_CLK pSDRAM_CLK]
set_output_delay -clock pSDRAM_CLK -max 1.5 [get_ports { o_sdram_cke o_sdram_cs_l o_sdram_ras_l o_sdram_cas_l o_sdram_we_l o_sdram_addr[*] o_sdram_ba[*] o_sdram_dq[*] o_sdram_dqml o_sdram_dqmh }]
set_output_delay -clock pSDRAM_CLK -min 0.8 [get_ports { o_sdram_cke o_sdram_cs_l o_sdram_ras_l o_sdram_cas_l o_sdram_we_l o_sdram_addr[*] o_sdram_ba[*] o_sdram_dq[*] o_sdram_dqml o_sdram_dqmh }]
set_input_delay -clock pSDRAM_CLK -max 5.4 [get_ports { o_sdram_dq[*] }]
set_input_delay -clock pSDRAM_CLK -min 2.7 [get_ports { o_sdram_dq[*] }]

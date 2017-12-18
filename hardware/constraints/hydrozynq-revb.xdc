
###################################
## Differential Signals (Bank 35) #
###################################

set_property IOSTANDARD LVDS_25 [get_ports {in1a_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in1a_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in1a_p[0]}]
set_property DIFF_TERM TRUE [get_ports {in1a_n[0]}]
set_property PACKAGE_PIN M14 [get_ports {in1a_p[0]}]
set_property PACKAGE_PIN M15 [get_ports {in1a_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {in1b_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in1b_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in1b_p[0]}]
set_property DIFF_TERM TRUE [get_ports {in1b_n[0]}]
set_property PACKAGE_PIN G19 [get_ports {in1b_p[0]}]
set_property PACKAGE_PIN G20 [get_ports {in1b_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {in2a_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in2a_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in2a_p[0]}]
set_property DIFF_TERM TRUE [get_ports {in2a_n[0]}]
set_property PACKAGE_PIN H16 [get_ports {in2a_p[0]}]
set_property PACKAGE_PIN H17 [get_ports {in2a_n[0]}]


set_property DIFF_TERM TRUE [get_ports {in2b_p[0]}]
set_property DIFF_TERM TRUE [get_ports {in2b_n[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in2b_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in2b_n[0]}]
set_property PACKAGE_PIN M17 [get_ports {in2b_p[0]}]
set_property PACKAGE_PIN M18 [get_ports {in2b_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {data_clk_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {data_clk_n[0]}]
set_property DIFF_TERM TRUE [get_ports {data_clk_p[0]}]
set_property DIFF_TERM TRUE [get_ports {data_clk_n[0]}]
set_property PACKAGE_PIN L19 [get_ports {data_clk_p[0]}]
set_property PACKAGE_PIN L20 [get_ports {data_clk_n[0]}]


##########################
## SPI Signals (Bank 35) #
##########################


set_property IOSTANDARD LVCMOS25 [get_ports {cs[0]}]
set_property IOSTANDARD LVCMOS25 [get_ports sck]
set_property PACKAGE_PIN K14 [get_ports {cs[0]}]
set_property PACKAGE_PIN J14 [get_ports sck]


set_property IOSTANDARD LVCMOS25 [get_ports mosi]
set_property IOSTANDARD LVCMOS25 [get_ports miso]
set_property PACKAGE_PIN G18 [get_ports mosi]
set_property PACKAGE_PIN G17 [get_ports miso]


###################################
## Differential Signals (Bank 34) #
###################################

set_property IOSTANDARD LVDS_25 [get_ports {enc_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {enc_n[0]}]
set_property PACKAGE_PIN P15 [get_ports {enc_p[0]}]
set_property PACKAGE_PIN P16 [get_ports {enc_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {fr_clk_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {fr_clk_n[0]}]
set_property DIFF_TERM TRUE [get_ports {fr_clk_p[0]}]
set_property PACKAGE_PIN V12 [get_ports {fr_clk_p[0]}]
set_property PACKAGE_PIN W13 [get_ports {fr_clk_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {in3a_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in3a_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in3a_p[0]}]
set_property PACKAGE_PIN P14 [get_ports {in3a_p[0]}]
set_property PACKAGE_PIN R14 [get_ports {in3a_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {in3b_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in3b_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in3b_p[0]}]
set_property PACKAGE_PIN V15 [get_ports {in3b_p[0]}]
set_property PACKAGE_PIN W15 [get_ports {in3b_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {in4a_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in4a_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in4a_p[0]}]
set_property PACKAGE_PIN N20 [get_ports {in4a_p[0]}]
set_property PACKAGE_PIN P20 [get_ports {in4a_n[0]}]


set_property IOSTANDARD LVDS_25 [get_ports {in4b_p[0]}]
set_property IOSTANDARD LVDS_25 [get_ports {in4b_n[0]}]
set_property DIFF_TERM TRUE [get_ports {in4b_p[0]}]
set_property PACKAGE_PIN V16 [get_ports {in4b_p[0]}]
set_property PACKAGE_PIN W16 [get_ports {in4b_n[0]}]


################################
# Clock and Timing Constraints #
################################

create_clock -period 50.000 -name DATA_CLK -waveform {12.500 37.500} [get_ports data_clk_p]
create_clock -period 200.000 -name FRAME_CLK -waveform {0.000 100.000} [get_ports fr_clk_p]

set_clock_groups -name adc_clocks -asynchronous -group {DATA_CLK FRAME_CLK}

set_input_delay -clock [get_clocks DATA_CLK] -clock_fall -min -add_delay 4.000 [get_ports {in**_p[0]}]
set_input_delay -clock [get_clocks DATA_CLK] -clock_fall -max -add_delay 21.000 [get_ports {in**_p[0]}]
set_input_delay -clock [get_clocks DATA_CLK] -min -add_delay 4.000 [get_ports {in**_p[0]}]
set_input_delay -clock [get_clocks DATA_CLK] -max -add_delay 21.000 [get_ports {in**_p[0]}]




################################################################
# This is a generated script based on design: design_1
#
# Though there are limitations about the generated script,
# the main purpose of this utility is to make learning
# IP Integrator Tcl commands easier.
################################################################

namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]

################################################################
# Check if script is running in correct Vivado version.
################################################################
set scripts_vivado_version 2017.2
set current_vivado_version [version -short]

if { [string first $scripts_vivado_version $current_vivado_version] == -1 } {
   puts ""
   catch {common::send_msg_id "BD_TCL-109" "ERROR" "This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Please run the script in Vivado <$scripts_vivado_version> then open the design in Vivado <$current_vivado_version>. Upgrade the design by running \"Tools => Report => Report IP Status...\", then run write_bd_tcl to create an updated script."}

   return 1
}

################################################################
# START
################################################################

# To test this script, run the following commands from Vivado Tcl console:
# source design_1_script.tcl

# If there is no project opened, this script will create a
# project, but make sure you do not have an existing project
# <./myproj/project_1.xpr> in the current working folder.

set list_projs [get_projects -quiet]
if { $list_projs eq "" } {
   create_project project_1 myproj -part xc7z010clg400-1
   set_property BOARD_PART em.avnet.com:microzed_7010:part0:1.0 [current_project]
}


# CHANGE DESIGN NAME HERE
set design_name design_1

# If you do not already have an existing IP Integrator design open,
# you can create a design using the following command:
#    create_bd_design $design_name

# Creating design if needed
set errMsg ""
set nRet 0

set cur_design [current_bd_design -quiet]
set list_cells [get_bd_cells -quiet]

if { ${design_name} eq "" } {
   # USE CASES:
   #    1) Design_name not set

   set errMsg "Please set the variable <design_name> to a non-empty value."
   set nRet 1

} elseif { ${cur_design} ne "" && ${list_cells} eq "" } {
   # USE CASES:
   #    2): Current design opened AND is empty AND names same.
   #    3): Current design opened AND is empty AND names diff; design_name NOT in project.
   #    4): Current design opened AND is empty AND names diff; design_name exists in project.

   if { $cur_design ne $design_name } {
      common::send_msg_id "BD_TCL-001" "INFO" "Changing value of <design_name> from <$design_name> to <$cur_design> since current design is empty."
      set design_name [get_property NAME $cur_design]
   }
   common::send_msg_id "BD_TCL-002" "INFO" "Constructing design in IPI design <$cur_design>..."

} elseif { ${cur_design} ne "" && $list_cells ne "" && $cur_design eq $design_name } {
   # USE CASES:
   #    5) Current design opened AND has components AND same names.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 1
} elseif { [get_files -quiet ${design_name}.bd] ne "" } {
   # USE CASES: 
   #    6) Current opened design, has components, but diff names, design_name exists in project.
   #    7) No opened design, design_name exists in project.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 2

} else {
   # USE CASES:
   #    8) No opened design, design_name not in project.
   #    9) Current opened design, has components, but diff names, design_name not in project.

   common::send_msg_id "BD_TCL-003" "INFO" "Currently there is no design <$design_name> in project, so creating one..."

   create_bd_design $design_name

   common::send_msg_id "BD_TCL-004" "INFO" "Making design <$design_name> as current_bd_design."
   current_bd_design $design_name

}

common::send_msg_id "BD_TCL-005" "INFO" "Currently the variable <design_name> is equal to \"$design_name\"."

if { $nRet != 0 } {
   catch {common::send_msg_id "BD_TCL-114" "ERROR" $errMsg}
   return $nRet
}

##################################################################
# DESIGN PROCs
##################################################################



# Procedure to create entire design; Provide argument to make
# procedure reusable. If parentCell is "", will use root.
proc create_root_design { parentCell } {

  variable script_folder

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_msg_id "BD_TCL-100" "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_msg_id "BD_TCL-101" "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj


  # Create interface ports
  set DDR [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 DDR ]
  set FIXED_IO [ create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 FIXED_IO ]

  # Create ports
  set cs [ create_bd_port -dir O -from 0 -to 0 cs ]
  set data_clk_n [ create_bd_port -dir I -from 0 -to 0 -type clk data_clk_n ]
  set_property -dict [ list \
CONFIG.FREQ_HZ {20000000} \
 ] $data_clk_n
  set data_clk_p [ create_bd_port -dir I -from 0 -to 0 -type clk data_clk_p ]
  set_property -dict [ list \
CONFIG.FREQ_HZ {20000000} \
 ] $data_clk_p
  set enc_n [ create_bd_port -dir O -from 0 -to 0 -type clk enc_n ]
  set enc_p [ create_bd_port -dir O -from 0 -to 0 -type clk enc_p ]
  set fr_clk_n [ create_bd_port -dir I -from 0 -to 0 -type clk fr_clk_n ]
  set_property -dict [ list \
CONFIG.FREQ_HZ {5000000} \
 ] $fr_clk_n
  set fr_clk_p [ create_bd_port -dir I -from 0 -to 0 -type clk fr_clk_p ]
  set_property -dict [ list \
CONFIG.FREQ_HZ {5000000} \
 ] $fr_clk_p
  set in1a_n [ create_bd_port -dir I -from 0 -to 0 -type data in1a_n ]
  set in1a_p [ create_bd_port -dir I -from 0 -to 0 -type data in1a_p ]
  set in1b_n [ create_bd_port -dir I -from 0 -to 0 -type data in1b_n ]
  set in1b_p [ create_bd_port -dir I -from 0 -to 0 -type data in1b_p ]
  set in2a_n [ create_bd_port -dir I -from 0 -to 0 -type data in2a_n ]
  set in2a_p [ create_bd_port -dir I -from 0 -to 0 -type data in2a_p ]
  set in2b_n [ create_bd_port -dir I -from 0 -to 0 -type data in2b_n ]
  set in2b_p [ create_bd_port -dir I -from 0 -to 0 -type data in2b_p ]
  set in3a_n [ create_bd_port -dir I -from 0 -to 0 -type data in3a_n ]
  set in3a_p [ create_bd_port -dir I -from 0 -to 0 -type data in3a_p ]
  set in3b_n [ create_bd_port -dir I -from 0 -to 0 -type data in3b_n ]
  set in3b_p [ create_bd_port -dir I -from 0 -to 0 -type data in3b_p ]
  set in4a_n [ create_bd_port -dir I -from 0 -to 0 -type data in4a_n ]
  set in4a_p [ create_bd_port -dir I -from 0 -to 0 -type data in4a_p ]
  set in4b_n [ create_bd_port -dir I -from 0 -to 0 -type data in4b_n ]
  set in4b_p [ create_bd_port -dir I -from 0 -to 0 -type data in4b_p ]
  set miso [ create_bd_port -dir I -type data miso ]
  set mosi [ create_bd_port -dir O -type data mosi ]
  set sck [ create_bd_port -dir O sck ]

  # Create instance: axi_dma_0, and set properties
  set axi_dma_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_0 ]
  set_property -dict [ list \
CONFIG.c_include_mm2s {0} \
CONFIG.c_include_s2mm {1} \
CONFIG.c_include_sg {0} \
CONFIG.c_micro_dma {0} \
CONFIG.c_sg_include_stscntrl_strm {0} \
CONFIG.c_sg_length_width {23} \
 ] $axi_dma_0

  # Create instance: axi_quad_spi_0, and set properties
  set axi_quad_spi_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_quad_spi:3.2 axi_quad_spi_0 ]
  set_property -dict [ list \
CONFIG.C_FIFO_DEPTH {0} \
CONFIG.C_NUM_TRANSFER_BITS {16} \
CONFIG.C_SCK_RATIO {16} \
CONFIG.C_TYPE_OF_AXI4_INTERFACE {0} \
CONFIG.C_USE_STARTUP {0} \
CONFIG.C_USE_STARTUP_INT {0} \
CONFIG.C_XIP_MODE {0} \
CONFIG.FIFO_INCLUDED {0} \
CONFIG.Multiples16 {1} \
 ] $axi_quad_spi_0

  # Create instance: axi_smc, and set properties
  set axi_smc [ create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 axi_smc ]
  set_property -dict [ list \
CONFIG.NUM_SI {1} \
 ] $axi_smc

  # Create instance: fifo_generator_0, and set properties
  set fifo_generator_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator:13.1 fifo_generator_0 ]
  set_property -dict [ list \
CONFIG.Clock_Type_AXI {Independent_Clock} \
CONFIG.Empty_Threshold_Assert_Value_axis {32765} \
CONFIG.Empty_Threshold_Assert_Value_rach {13} \
CONFIG.Empty_Threshold_Assert_Value_rdch {1021} \
CONFIG.Empty_Threshold_Assert_Value_wach {13} \
CONFIG.Empty_Threshold_Assert_Value_wdch {1021} \
CONFIG.Empty_Threshold_Assert_Value_wrch {13} \
CONFIG.Enable_TLAST {true} \
CONFIG.FIFO_Application_Type_axis {Packet_FIFO} \
CONFIG.FIFO_Implementation_axis {Independent_Clocks_Block_RAM} \
CONFIG.FIFO_Implementation_rach {Independent_Clocks_Distributed_RAM} \
CONFIG.FIFO_Implementation_rdch {Independent_Clocks_Block_RAM} \
CONFIG.FIFO_Implementation_wach {Independent_Clocks_Distributed_RAM} \
CONFIG.FIFO_Implementation_wdch {Independent_Clocks_Block_RAM} \
CONFIG.FIFO_Implementation_wrch {Independent_Clocks_Distributed_RAM} \
CONFIG.Fifo_Implementation {Independent_Clocks_Block_RAM} \
CONFIG.Full_Flags_Reset_Value {1} \
CONFIG.Full_Threshold_Assert_Value {1021} \
CONFIG.Full_Threshold_Assert_Value_axis {32767} \
CONFIG.Full_Threshold_Assert_Value_rach {15} \
CONFIG.Full_Threshold_Assert_Value_wach {15} \
CONFIG.Full_Threshold_Assert_Value_wrch {15} \
CONFIG.Full_Threshold_Negate_Value {1020} \
CONFIG.INTERFACE_TYPE {AXI_STREAM} \
CONFIG.Input_Depth_axis {32768} \
CONFIG.Reset_Type {Asynchronous_Reset} \
CONFIG.TDATA_NUM_BYTES {4} \
CONFIG.TKEEP_WIDTH {4} \
CONFIG.TSTRB_WIDTH {4} \
 ] $fifo_generator_0

  # Create instance: ila_1, and set properties
  set ila_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:ila:6.2 ila_1 ]
  set_property -dict [ list \
CONFIG.C_NUM_OF_PROBES {9} \
CONFIG.C_SLOT_0_AXI_PROTOCOL {AXI4S} \
 ] $ila_1

  # Create instance: processing_system7_0, and set properties
  set processing_system7_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0 ]
  set_property -dict [ list \
CONFIG.PCW_APU_CLK_RATIO_ENABLE {6:2:1} \
CONFIG.PCW_APU_PERIPHERAL_FREQMHZ {667} \
CONFIG.PCW_CPU_PERIPHERAL_CLKSRC {ARM PLL} \
CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
CONFIG.PCW_DDR_PERIPHERAL_CLKSRC {DDR PLL} \
CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27} \
CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {1} \
CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53} \
CONFIG.PCW_ENET0_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_ENET0_PERIPHERAL_FREQMHZ {1000 Mbps} \
CONFIG.PCW_ENET0_RESET_ENABLE {0} \
CONFIG.PCW_EN_CLK0_PORT {1} \
CONFIG.PCW_EN_CLK1_PORT {0} \
CONFIG.PCW_EN_CLK2_PORT {0} \
CONFIG.PCW_EN_CLK3_PORT {0} \
CONFIG.PCW_EN_DDR {1} \
CONFIG.PCW_EN_RST0_PORT {1} \
CONFIG.PCW_EN_RST1_PORT {0} \
CONFIG.PCW_EN_RST2_PORT {0} \
CONFIG.PCW_EN_RST3_PORT {0} \
CONFIG.PCW_FCLK0_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK1_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK2_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK3_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_FCLK_CLK0_BUF {TRUE} \
CONFIG.PCW_FCLK_CLK1_BUF {FALSE} \
CONFIG.PCW_FCLK_CLK2_BUF {FALSE} \
CONFIG.PCW_FCLK_CLK3_BUF {FALSE} \
CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {100} \
CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ {33.333333} \
CONFIG.PCW_FPGA3_PERIPHERAL_FREQMHZ {50} \
CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {0} \
CONFIG.PCW_GPIO_MIO_GPIO_ENABLE {1} \
CONFIG.PCW_GPIO_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_I2C_RESET_ENABLE {0} \
CONFIG.PCW_MIO_0_PULLUP {disabled} \
CONFIG.PCW_MIO_0_SLEW {slow} \
CONFIG.PCW_MIO_10_PULLUP {disabled} \
CONFIG.PCW_MIO_10_SLEW {slow} \
CONFIG.PCW_MIO_11_PULLUP {disabled} \
CONFIG.PCW_MIO_11_SLEW {slow} \
CONFIG.PCW_MIO_12_PULLUP {disabled} \
CONFIG.PCW_MIO_12_SLEW {slow} \
CONFIG.PCW_MIO_13_PULLUP {disabled} \
CONFIG.PCW_MIO_13_SLEW {slow} \
CONFIG.PCW_MIO_14_PULLUP {disabled} \
CONFIG.PCW_MIO_14_SLEW {slow} \
CONFIG.PCW_MIO_15_PULLUP {disabled} \
CONFIG.PCW_MIO_15_SLEW {slow} \
CONFIG.PCW_MIO_16_PULLUP {disabled} \
CONFIG.PCW_MIO_16_SLEW {slow} \
CONFIG.PCW_MIO_17_PULLUP {disabled} \
CONFIG.PCW_MIO_17_SLEW {slow} \
CONFIG.PCW_MIO_18_PULLUP {disabled} \
CONFIG.PCW_MIO_18_SLEW {slow} \
CONFIG.PCW_MIO_19_PULLUP {disabled} \
CONFIG.PCW_MIO_19_SLEW {slow} \
CONFIG.PCW_MIO_1_PULLUP {disabled} \
CONFIG.PCW_MIO_1_SLEW {slow} \
CONFIG.PCW_MIO_20_PULLUP {disabled} \
CONFIG.PCW_MIO_20_SLEW {slow} \
CONFIG.PCW_MIO_21_PULLUP {disabled} \
CONFIG.PCW_MIO_21_SLEW {slow} \
CONFIG.PCW_MIO_22_PULLUP {disabled} \
CONFIG.PCW_MIO_22_SLEW {slow} \
CONFIG.PCW_MIO_23_PULLUP {disabled} \
CONFIG.PCW_MIO_23_SLEW {slow} \
CONFIG.PCW_MIO_24_PULLUP {disabled} \
CONFIG.PCW_MIO_24_SLEW {slow} \
CONFIG.PCW_MIO_25_PULLUP {disabled} \
CONFIG.PCW_MIO_25_SLEW {slow} \
CONFIG.PCW_MIO_26_PULLUP {disabled} \
CONFIG.PCW_MIO_26_SLEW {slow} \
CONFIG.PCW_MIO_27_PULLUP {disabled} \
CONFIG.PCW_MIO_27_SLEW {slow} \
CONFIG.PCW_MIO_28_PULLUP {disabled} \
CONFIG.PCW_MIO_28_SLEW {slow} \
CONFIG.PCW_MIO_29_PULLUP {disabled} \
CONFIG.PCW_MIO_29_SLEW {slow} \
CONFIG.PCW_MIO_2_PULLUP {disabled} \
CONFIG.PCW_MIO_2_SLEW {slow} \
CONFIG.PCW_MIO_30_PULLUP {disabled} \
CONFIG.PCW_MIO_30_SLEW {slow} \
CONFIG.PCW_MIO_31_PULLUP {disabled} \
CONFIG.PCW_MIO_31_SLEW {slow} \
CONFIG.PCW_MIO_32_PULLUP {disabled} \
CONFIG.PCW_MIO_32_SLEW {slow} \
CONFIG.PCW_MIO_33_PULLUP {disabled} \
CONFIG.PCW_MIO_33_SLEW {slow} \
CONFIG.PCW_MIO_34_PULLUP {disabled} \
CONFIG.PCW_MIO_34_SLEW {slow} \
CONFIG.PCW_MIO_35_PULLUP {disabled} \
CONFIG.PCW_MIO_35_SLEW {slow} \
CONFIG.PCW_MIO_36_PULLUP {disabled} \
CONFIG.PCW_MIO_36_SLEW {slow} \
CONFIG.PCW_MIO_37_PULLUP {disabled} \
CONFIG.PCW_MIO_37_SLEW {slow} \
CONFIG.PCW_MIO_38_PULLUP {disabled} \
CONFIG.PCW_MIO_38_SLEW {slow} \
CONFIG.PCW_MIO_39_PULLUP {disabled} \
CONFIG.PCW_MIO_39_SLEW {slow} \
CONFIG.PCW_MIO_3_PULLUP {disabled} \
CONFIG.PCW_MIO_3_SLEW {slow} \
CONFIG.PCW_MIO_40_PULLUP {disabled} \
CONFIG.PCW_MIO_40_SLEW {slow} \
CONFIG.PCW_MIO_41_PULLUP {disabled} \
CONFIG.PCW_MIO_41_SLEW {slow} \
CONFIG.PCW_MIO_42_PULLUP {disabled} \
CONFIG.PCW_MIO_42_SLEW {slow} \
CONFIG.PCW_MIO_43_PULLUP {disabled} \
CONFIG.PCW_MIO_43_SLEW {slow} \
CONFIG.PCW_MIO_44_PULLUP {disabled} \
CONFIG.PCW_MIO_44_SLEW {slow} \
CONFIG.PCW_MIO_45_PULLUP {disabled} \
CONFIG.PCW_MIO_45_SLEW {slow} \
CONFIG.PCW_MIO_46_PULLUP {disabled} \
CONFIG.PCW_MIO_46_SLEW {slow} \
CONFIG.PCW_MIO_47_PULLUP {disabled} \
CONFIG.PCW_MIO_47_SLEW {slow} \
CONFIG.PCW_MIO_48_PULLUP {disabled} \
CONFIG.PCW_MIO_48_SLEW {slow} \
CONFIG.PCW_MIO_49_PULLUP {disabled} \
CONFIG.PCW_MIO_49_SLEW {slow} \
CONFIG.PCW_MIO_4_PULLUP {disabled} \
CONFIG.PCW_MIO_4_SLEW {slow} \
CONFIG.PCW_MIO_50_PULLUP {disabled} \
CONFIG.PCW_MIO_50_SLEW {slow} \
CONFIG.PCW_MIO_51_PULLUP {disabled} \
CONFIG.PCW_MIO_51_SLEW {slow} \
CONFIG.PCW_MIO_52_PULLUP {disabled} \
CONFIG.PCW_MIO_52_SLEW {slow} \
CONFIG.PCW_MIO_53_PULLUP {disabled} \
CONFIG.PCW_MIO_53_SLEW {slow} \
CONFIG.PCW_MIO_5_PULLUP {disabled} \
CONFIG.PCW_MIO_5_SLEW {slow} \
CONFIG.PCW_MIO_6_PULLUP {disabled} \
CONFIG.PCW_MIO_6_SLEW {slow} \
CONFIG.PCW_MIO_7_PULLUP {disabled} \
CONFIG.PCW_MIO_7_SLEW {slow} \
CONFIG.PCW_MIO_8_PULLUP {disabled} \
CONFIG.PCW_MIO_8_SLEW {slow} \
CONFIG.PCW_MIO_9_PULLUP {disabled} \
CONFIG.PCW_MIO_9_SLEW {slow} \
CONFIG.PCW_PACKAGE_NAME {clg400} \
CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 3.3V} \
CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V} \
CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE {1} \
CONFIG.PCW_QSPI_GRP_FBCLK_IO {MIO 8} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE {1} \
CONFIG.PCW_QSPI_GRP_SINGLE_SS_IO {MIO 1 .. 6} \
CONFIG.PCW_QSPI_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_QSPI_PERIPHERAL_FREQMHZ {200} \
CONFIG.PCW_SD0_GRP_CD_ENABLE {1} \
CONFIG.PCW_SD0_GRP_CD_IO {MIO 46} \
CONFIG.PCW_SD0_GRP_WP_ENABLE {1} \
CONFIG.PCW_SD0_GRP_WP_IO {MIO 50} \
CONFIG.PCW_SD0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_SD0_SD0_IO {MIO 40 .. 45} \
CONFIG.PCW_SDIO_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ {50} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC0_CLK0_PERIPHERAL_FREQMHZ {111.111115} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC0_CLK1_PERIPHERAL_FREQMHZ {111.111115} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_CLKSRC {CPU_1X} \
CONFIG.PCW_TTC0_CLK2_PERIPHERAL_FREQMHZ {111.111115} \
CONFIG.PCW_TTC0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_TTC0_TTC0_IO {EMIO} \
CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_UART1_UART1_IO {MIO 48 .. 49} \
CONFIG.PCW_UART_PERIPHERAL_CLKSRC {IO PLL} \
CONFIG.PCW_UART_PERIPHERAL_FREQMHZ {50} \
CONFIG.PCW_UIPARAM_ACT_DDR_FREQ_MHZ {533.333374} \
CONFIG.PCW_UIPARAM_DDR_BL {8} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 {0.294} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 {0.298} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY2 {0.338} \
CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY3 {0.334} \
CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {32 Bit} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_0_LENGTH_MM {39.7} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_1_LENGTH_MM {39.7} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_2_LENGTH_MM {54.14} \
CONFIG.PCW_UIPARAM_DDR_CLOCK_3_LENGTH_MM {54.14} \
CONFIG.PCW_UIPARAM_DDR_CWL {6} \
CONFIG.PCW_UIPARAM_DDR_DEVICE_CAPACITY {4096 MBits} \
CONFIG.PCW_UIPARAM_DDR_DQS_0_LENGTH_MM {50.05} \
CONFIG.PCW_UIPARAM_DDR_DQS_1_LENGTH_MM {50.43} \
CONFIG.PCW_UIPARAM_DDR_DQS_2_LENGTH_MM {50.10} \
CONFIG.PCW_UIPARAM_DDR_DQS_3_LENGTH_MM {50.01} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 {-0.073} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 {-0.072} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2 {0.024} \
CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3 {0.023} \
CONFIG.PCW_UIPARAM_DDR_DQ_0_LENGTH_MM {49.59} \
CONFIG.PCW_UIPARAM_DDR_DQ_1_LENGTH_MM {51.74} \
CONFIG.PCW_UIPARAM_DDR_DQ_2_LENGTH_MM {50.32} \
CONFIG.PCW_UIPARAM_DDR_DQ_3_LENGTH_MM {48.55} \
CONFIG.PCW_UIPARAM_DDR_DRAM_WIDTH {16 Bits} \
CONFIG.PCW_UIPARAM_DDR_MEMORY_TYPE {DDR 3 (Low Voltage)} \
CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125} \
CONFIG.PCW_UIPARAM_DDR_SPEED_BIN {DDR3_1066F} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE {1} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE {1} \
CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL {1} \
CONFIG.PCW_UIPARAM_DDR_T_FAW {40.0} \
CONFIG.PCW_UIPARAM_DDR_T_RAS_MIN {35.0} \
CONFIG.PCW_UIPARAM_DDR_T_RC {48.75} \
CONFIG.PCW_UIPARAM_DDR_USE_INTERNAL_VREF {1} \
CONFIG.PCW_USB0_PERIPHERAL_ENABLE {1} \
CONFIG.PCW_USB0_PERIPHERAL_FREQMHZ {60} \
CONFIG.PCW_USB0_RESET_ENABLE {1} \
CONFIG.PCW_USB0_RESET_IO {MIO 7} \
CONFIG.PCW_USB0_USB0_IO {MIO 28 .. 39} \
CONFIG.PCW_USE_M_AXI_GP0 {1} \
CONFIG.PCW_USE_M_AXI_GP1 {0} \
CONFIG.PCW_USE_S_AXI_HP0 {1} \
 ] $processing_system7_0

  # Create instance: ps7_0_axi_periph, and set properties
  set ps7_0_axi_periph [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 ps7_0_axi_periph ]
  set_property -dict [ list \
CONFIG.NUM_MI {5} \
 ] $ps7_0_axi_periph

  # Create instance: quad_adc_0, and set properties
  set quad_adc_0 [ create_bd_cell -type ip -vlnv user.org:user:quad_adc:1.3 quad_adc_0 ]

  # Create instance: rst_ps7_0_100M, and set properties
  set rst_ps7_0_100M [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_ps7_0_100M ]

  # Create instance: util_ds_buf_0, and set properties
  set util_ds_buf_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_0 ]

  # Create instance: util_ds_buf_1, and set properties
  set util_ds_buf_1 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_1 ]

  # Create instance: util_ds_buf_2, and set properties
  set util_ds_buf_2 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_2 ]

  # Create instance: util_ds_buf_3, and set properties
  set util_ds_buf_3 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_3 ]

  # Create instance: util_ds_buf_4, and set properties
  set util_ds_buf_4 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_4 ]

  # Create instance: util_ds_buf_5, and set properties
  set util_ds_buf_5 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_5 ]

  # Create instance: util_ds_buf_6, and set properties
  set util_ds_buf_6 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_6 ]

  # Create instance: util_ds_buf_7, and set properties
  set util_ds_buf_7 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_7 ]

  # Create instance: util_ds_buf_8, and set properties
  set util_ds_buf_8 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_8 ]

  # Create instance: util_ds_buf_9, and set properties
  set util_ds_buf_9 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_9 ]

  # Create instance: util_ds_buf_10, and set properties
  set util_ds_buf_10 [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 util_ds_buf_10 ]
  set_property -dict [ list \
CONFIG.C_BUF_TYPE {OBUFDS} \
 ] $util_ds_buf_10

  # Create instance: xadc_wiz_0, and set properties
  set xadc_wiz_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:xadc_wiz:3.3 xadc_wiz_0 ]
  set_property -dict [ list \
CONFIG.ADC_CONVERSION_RATE {154} \
CONFIG.DCLK_FREQUENCY {8} \
CONFIG.ENABLE_TEMP_BUS {false} \
CONFIG.ENABLE_VCCDDRO_ALARM {false} \
CONFIG.ENABLE_VCCPAUX_ALARM {false} \
CONFIG.ENABLE_VCCPINT_ALARM {false} \
CONFIG.OT_ALARM {false} \
CONFIG.POWER_DOWN_ADCB {true} \
CONFIG.SINGLE_CHANNEL_ENABLE_CALIBRATION {true} \
CONFIG.USER_TEMP_ALARM {false} \
CONFIG.VCCAUX_ALARM {false} \
CONFIG.VCCINT_ALARM {false} \
 ] $xadc_wiz_0

  # Create interface connections
  connect_bd_intf_net -intf_net axi_dma_0_M_AXI_S2MM [get_bd_intf_pins axi_dma_0/M_AXI_S2MM] [get_bd_intf_pins axi_smc/S00_AXI]
  connect_bd_intf_net -intf_net axi_smc_M00_AXI [get_bd_intf_pins axi_smc/M00_AXI] [get_bd_intf_pins processing_system7_0/S_AXI_HP0]
  connect_bd_intf_net -intf_net fifo_generator_0_M_AXIS [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM] [get_bd_intf_pins fifo_generator_0/M_AXIS]
connect_bd_intf_net -intf_net [get_bd_intf_nets fifo_generator_0_M_AXIS] [get_bd_intf_pins fifo_generator_0/M_AXIS] [get_bd_intf_pins ila_1/SLOT_0_AXIS]
  connect_bd_intf_net -intf_net processing_system7_0_DDR [get_bd_intf_ports DDR] [get_bd_intf_pins processing_system7_0/DDR]
  connect_bd_intf_net -intf_net processing_system7_0_FIXED_IO [get_bd_intf_ports FIXED_IO] [get_bd_intf_pins processing_system7_0/FIXED_IO]
  connect_bd_intf_net -intf_net processing_system7_0_M_AXI_GP0 [get_bd_intf_pins processing_system7_0/M_AXI_GP0] [get_bd_intf_pins ps7_0_axi_periph/S00_AXI]
  connect_bd_intf_net -intf_net ps7_0_axi_periph_M00_AXI [get_bd_intf_pins axi_quad_spi_0/AXI_LITE] [get_bd_intf_pins ps7_0_axi_periph/M00_AXI]
  connect_bd_intf_net -intf_net ps7_0_axi_periph_M01_AXI [get_bd_intf_pins ps7_0_axi_periph/M01_AXI] [get_bd_intf_pins xadc_wiz_0/s_axi_lite]
  connect_bd_intf_net -intf_net ps7_0_axi_periph_M02_AXI [get_bd_intf_pins axi_dma_0/S_AXI_LITE] [get_bd_intf_pins ps7_0_axi_periph/M02_AXI]
  connect_bd_intf_net -intf_net ps7_0_axi_periph_M03_AXI [get_bd_intf_pins ps7_0_axi_periph/M03_AXI] [get_bd_intf_pins quad_adc_0/S00_AXI]
  connect_bd_intf_net -intf_net quad_adc_0_M00_AXIS [get_bd_intf_pins fifo_generator_0/S_AXIS] [get_bd_intf_pins quad_adc_0/M00_AXIS]

  # Create port connections
  connect_bd_net -net CH_1_A [get_bd_pins quad_adc_0/CH_1_A] [get_bd_pins util_ds_buf_2/IBUF_OUT]
  connect_bd_net -net CH_1_B [get_bd_pins quad_adc_0/CH_1_B] [get_bd_pins util_ds_buf_3/IBUF_OUT]
  connect_bd_net -net CH_2_A [get_bd_pins quad_adc_0/CH_2_A] [get_bd_pins util_ds_buf_4/IBUF_OUT]
  connect_bd_net -net CH_2_B [get_bd_pins quad_adc_0/CH_2_B] [get_bd_pins util_ds_buf_5/IBUF_OUT]
  connect_bd_net -net CH_3_A [get_bd_pins quad_adc_0/CH_3_A] [get_bd_pins util_ds_buf_6/IBUF_OUT]
  connect_bd_net -net CH_3_B [get_bd_pins quad_adc_0/CH_3_B] [get_bd_pins util_ds_buf_7/IBUF_OUT]
  connect_bd_net -net CH_4_A [get_bd_pins quad_adc_0/CH_4_A] [get_bd_pins util_ds_buf_8/IBUF_OUT]
  connect_bd_net -net CH_4_B [get_bd_pins quad_adc_0/CH_4_B] [get_bd_pins util_ds_buf_9/IBUF_OUT]
  connect_bd_net -net DATA_CLK [get_bd_pins fifo_generator_0/s_aclk] [get_bd_pins quad_adc_0/DATA_CLK] [get_bd_pins quad_adc_0/m00_axis_aclk] [get_bd_pins util_ds_buf_0/IBUF_OUT]
  connect_bd_net -net ENCODE_CLK [get_bd_pins quad_adc_0/ENCODE_CLK] [get_bd_pins util_ds_buf_10/OBUF_IN]
  connect_bd_net -net FRAME_CLK [get_bd_pins quad_adc_0/FRAME_CLK] [get_bd_pins util_ds_buf_1/IBUF_OUT]
  connect_bd_net -net IBUF_DS_N_1_1 [get_bd_ports fr_clk_n] [get_bd_pins util_ds_buf_1/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_2 [get_bd_ports data_clk_n] [get_bd_pins util_ds_buf_0/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_2_1 [get_bd_ports in1a_n] [get_bd_pins util_ds_buf_2/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_3_1 [get_bd_ports in1b_n] [get_bd_pins util_ds_buf_3/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_4_1 [get_bd_ports in2a_n] [get_bd_pins util_ds_buf_4/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_5_1 [get_bd_ports in2b_n] [get_bd_pins util_ds_buf_5/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_6_1 [get_bd_ports in3a_n] [get_bd_pins util_ds_buf_6/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_7_1 [get_bd_ports in3b_n] [get_bd_pins util_ds_buf_7/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_8_1 [get_bd_ports in4a_n] [get_bd_pins util_ds_buf_8/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_N_9_1 [get_bd_ports in4b_n] [get_bd_pins util_ds_buf_9/IBUF_DS_N]
  connect_bd_net -net IBUF_DS_P_1_1 [get_bd_ports fr_clk_p] [get_bd_pins util_ds_buf_1/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_2_1 [get_bd_ports in1a_p] [get_bd_pins util_ds_buf_2/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_3 [get_bd_ports data_clk_p] [get_bd_pins util_ds_buf_0/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_3_1 [get_bd_ports in1b_p] [get_bd_pins util_ds_buf_3/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_4_1 [get_bd_ports in2a_p] [get_bd_pins util_ds_buf_4/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_5_1 [get_bd_ports in2b_p] [get_bd_pins util_ds_buf_5/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_6_1 [get_bd_ports in3a_p] [get_bd_pins util_ds_buf_6/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_7_1 [get_bd_ports in3b_p] [get_bd_pins util_ds_buf_7/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_8_1 [get_bd_ports in4a_p] [get_bd_pins util_ds_buf_8/IBUF_DS_P]
  connect_bd_net -net IBUF_DS_P_9_1 [get_bd_ports in4b_p] [get_bd_pins util_ds_buf_9/IBUF_DS_P]
  connect_bd_net -net axi_quad_spi_0_io0_o [get_bd_ports mosi] [get_bd_pins axi_quad_spi_0/io0_o]
  connect_bd_net -net axi_quad_spi_0_sck_o [get_bd_ports sck] [get_bd_pins axi_quad_spi_0/sck_o]
  connect_bd_net -net axi_quad_spi_0_ss_o [get_bd_ports cs] [get_bd_pins axi_quad_spi_0/ss_o]
  connect_bd_net -net miso_1 [get_bd_ports miso] [get_bd_pins axi_quad_spi_0/io1_i]
  connect_bd_net -net processing_system7_0_FCLK_CLK0 [get_bd_pins axi_dma_0/m_axi_s2mm_aclk] [get_bd_pins axi_dma_0/s_axi_lite_aclk] [get_bd_pins axi_quad_spi_0/ext_spi_clk] [get_bd_pins axi_quad_spi_0/s_axi_aclk] [get_bd_pins axi_smc/aclk] [get_bd_pins fifo_generator_0/m_aclk] [get_bd_pins ila_1/clk] [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins processing_system7_0/M_AXI_GP0_ACLK] [get_bd_pins processing_system7_0/S_AXI_HP0_ACLK] [get_bd_pins ps7_0_axi_periph/ACLK] [get_bd_pins ps7_0_axi_periph/M00_ACLK] [get_bd_pins ps7_0_axi_periph/M01_ACLK] [get_bd_pins ps7_0_axi_periph/M02_ACLK] [get_bd_pins ps7_0_axi_periph/M03_ACLK] [get_bd_pins ps7_0_axi_periph/M04_ACLK] [get_bd_pins ps7_0_axi_periph/S00_ACLK] [get_bd_pins quad_adc_0/s00_axi_aclk] [get_bd_pins rst_ps7_0_100M/slowest_sync_clk] [get_bd_pins xadc_wiz_0/s_axi_aclk]
  connect_bd_net -net processing_system7_0_FCLK_RESET0_N [get_bd_pins processing_system7_0/FCLK_RESET0_N] [get_bd_pins rst_ps7_0_100M/ext_reset_in]
  connect_bd_net -net rst_ps7_0_100M_interconnect_aresetn [get_bd_pins ps7_0_axi_periph/ARESETN] [get_bd_pins rst_ps7_0_100M/interconnect_aresetn]
  connect_bd_net -net rst_ps7_0_100M_peripheral_aresetn [get_bd_pins axi_dma_0/axi_resetn] [get_bd_pins axi_quad_spi_0/s_axi_aresetn] [get_bd_pins axi_smc/aresetn] [get_bd_pins fifo_generator_0/s_aresetn] [get_bd_pins ps7_0_axi_periph/M00_ARESETN] [get_bd_pins ps7_0_axi_periph/M01_ARESETN] [get_bd_pins ps7_0_axi_periph/M02_ARESETN] [get_bd_pins ps7_0_axi_periph/M03_ARESETN] [get_bd_pins ps7_0_axi_periph/M04_ARESETN] [get_bd_pins ps7_0_axi_periph/S00_ARESETN] [get_bd_pins quad_adc_0/m00_axis_aresetn] [get_bd_pins quad_adc_0/s00_axi_aresetn] [get_bd_pins rst_ps7_0_100M/peripheral_aresetn] [get_bd_pins xadc_wiz_0/s_axi_aresetn]
  connect_bd_net -net util_ds_buf_10_OBUF_DS_N [get_bd_ports enc_n] [get_bd_pins util_ds_buf_10/OBUF_DS_N]
  connect_bd_net -net util_ds_buf_10_OBUF_DS_P [get_bd_ports enc_p] [get_bd_pins util_ds_buf_10/OBUF_DS_P]

  # Create address segments
  create_bd_addr_seg -range 0x40000000 -offset 0x00000000 [get_bd_addr_spaces axi_dma_0/Data_S2MM] [get_bd_addr_segs processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM] SEG_processing_system7_0_HP0_DDR_LOWOCM
  create_bd_addr_seg -range 0x00010000 -offset 0x40400000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_dma_0/S_AXI_LITE/Reg] SEG_axi_dma_0_Reg
  create_bd_addr_seg -range 0x00010000 -offset 0x41E00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs axi_quad_spi_0/AXI_LITE/Reg] SEG_axi_quad_spi_0_Reg
  create_bd_addr_seg -range 0x00010000 -offset 0x43C10000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs quad_adc_0/S00_AXI/S00_AXI_reg] SEG_quad_adc_0_S00_AXI_reg
  create_bd_addr_seg -range 0x00010000 -offset 0x43C00000 [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs xadc_wiz_0/s_axi_lite/Reg] SEG_xadc_wiz_0_Reg


  # Restore current instance
  current_bd_instance $oldCurInst

  save_bd_design
}
# End of create_root_design()


##################################################################
# MAIN FLOW
##################################################################

create_root_design ""




`timescale 1 ns / 1 ps

    module quad_adc_v1_0 #
    (
        // Users to add parameters here

        // User parameters ends
        // Do not modify the parameters beyond this line


        // Parameters of Axi Slave Bus Interface S00_AXI
        parameter integer C_S00_AXI_DATA_WIDTH  = 32,
        parameter integer C_S00_AXI_ADDR_WIDTH  = 4,

        // Parameters of Axi Master Bus Interface M00_AXIS
        parameter integer C_M00_AXIS_TDATA_WIDTH    = 32,
        parameter integer C_M00_AXIS_START_COUNT    = 32,

        // Parameters of Axi Slave Bus Interface S_AXI_INTR
        parameter integer C_S_AXI_INTR_DATA_WIDTH   = 32,
        parameter integer C_S_AXI_INTR_ADDR_WIDTH   = 5,
        parameter integer C_NUM_OF_INTR = 1,
        parameter  C_INTR_SENSITIVITY   = 32'hFFFFFFFF,
        parameter  C_INTR_ACTIVE_STATE  = 32'hFFFFFFFF,
        parameter integer C_IRQ_SENSITIVITY = 1,
        parameter integer C_IRQ_ACTIVE_STATE    = 1
    )
    (
        // Users to add ports here

        //ADC
        output wire ENCODE_CLK,
        input wire DATA_CLK,
        input wire FRAME_CLK,
        input wire CH_1_A, CH_1_B,
        input wire CH_2_A, CH_2_B,
        input wire CH_3_A, CH_3_B,
        input wire CH_4_A, CH_4_B,

        // User ports ends
        // Do not modify the ports beyond this line


        // Ports of Axi Slave Bus Interface S00_AXI
        input wire  s00_axi_aclk,
        input wire  s00_axi_aresetn,
        input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
        input wire [2 : 0] s00_axi_awprot,
        input wire  s00_axi_awvalid,
        output wire  s00_axi_awready,
        input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
        input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
        input wire  s00_axi_wvalid,
        output wire  s00_axi_wready,
        output wire [1 : 0] s00_axi_bresp,
        output wire  s00_axi_bvalid,
        input wire  s00_axi_bready,
        input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
        input wire [2 : 0] s00_axi_arprot,
        input wire  s00_axi_arvalid,
        output wire  s00_axi_arready,
        output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
        output wire [1 : 0] s00_axi_rresp,
        output wire  s00_axi_rvalid,
        input wire  s00_axi_rready,

        // Ports of Axi Master Bus Interface M00_AXIS
        input wire  m00_axis_aclk,
        input wire  m00_axis_aresetn,
        output wire  m00_axis_tvalid,
        output wire [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata,
        output wire [(C_M00_AXIS_TDATA_WIDTH/8)-1 : 0] m00_axis_tstrb,
        output wire  m00_axis_tlast,
        input wire  m00_axis_tready,

        // Ports of Axi Slave Bus Interface S_AXI_INTR
        input wire  s_axi_intr_aclk,
        input wire  s_axi_intr_aresetn,
        input wire [C_S_AXI_INTR_ADDR_WIDTH-1 : 0] s_axi_intr_awaddr,
        input wire [2 : 0] s_axi_intr_awprot,
        input wire  s_axi_intr_awvalid,
        output wire  s_axi_intr_awready,
        input wire [C_S_AXI_INTR_DATA_WIDTH-1 : 0] s_axi_intr_wdata,
        input wire [(C_S_AXI_INTR_DATA_WIDTH/8)-1 : 0] s_axi_intr_wstrb,
        input wire  s_axi_intr_wvalid,
        output wire  s_axi_intr_wready,
        output wire [1 : 0] s_axi_intr_bresp,
        output wire  s_axi_intr_bvalid,
        input wire  s_axi_intr_bready,
        input wire [C_S_AXI_INTR_ADDR_WIDTH-1 : 0] s_axi_intr_araddr,
        input wire [2 : 0] s_axi_intr_arprot,
        input wire  s_axi_intr_arvalid,
        output wire  s_axi_intr_arready,
        output wire [C_S_AXI_INTR_DATA_WIDTH-1 : 0] s_axi_intr_rdata,
        output wire [1 : 0] s_axi_intr_rresp,
        output wire  s_axi_intr_rvalid,
        input wire  s_axi_intr_rready,
        output wire  irq
    );

    // interconnects
    wire [13:0] AXI_CH_1_DATA, AXI_CH_2_DATA, AXI_CH_3_DATA, AXI_CH_4_DATA;
    wire [13:0] ADC_CH_1_DATA, ADC_CH_2_DATA, ADC_CH_3_DATA, ADC_CH_4_DATA;
    wire AXI_DATA_VALID;
    wire [C_S00_AXI_DATA_WIDTH-1 : 0] ENCODE_CLK_DIV;

// Instantiation of Axi Bus Interface S00_AXI
    quad_adc_v1_0_S00_AXI # (
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) quad_adc_v1_0_S00_AXI_inst (
        // output registers
        .ENCODE_CLK_DIV(ENCODE_CLK_DIV),

        // axi bus ports
        .S_AXI_ACLK(s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR(s00_axi_awaddr),
        .S_AXI_AWPROT(s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA(s00_axi_wdata),
        .S_AXI_WSTRB(s00_axi_wstrb),
        .S_AXI_WVALID(s00_axi_wvalid),
        .S_AXI_WREADY(s00_axi_wready),
        .S_AXI_BRESP(s00_axi_bresp),
        .S_AXI_BVALID(s00_axi_bvalid),
        .S_AXI_BREADY(s00_axi_bready),
        .S_AXI_ARADDR(s00_axi_araddr),
        .S_AXI_ARPROT(s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA(s00_axi_rdata),
        .S_AXI_RRESP(s00_axi_rresp),
        .S_AXI_RVALID(s00_axi_rvalid),
        .S_AXI_RREADY(s00_axi_rready)
    );

// Instantiation of Axi Bus Interface M00_AXIS
    quad_adc_v1_0_M00_AXIS # (
        .C_M_AXIS_TDATA_WIDTH(C_M00_AXIS_TDATA_WIDTH)
    ) quad_adc_v1_0_M00_AXIS_inst (
        // user ports
        .CH_A_DATA_IN({2'b0,AXI_CH_1_DATA}),
        .CH_B_DATA_IN({2'b0,AXI_CH_2_DATA}),
        .CH_C_DATA_IN({2'b0,AXI_CH_3_DATA}),
        .CH_D_DATA_IN({2'b0,AXI_CH_4_DATA}),
        .DATA_IN_VALID(AXI_DATA_VALID),

        // axi bus ports
        .M_AXIS_ACLK(m00_axis_aclk),
        .M_AXIS_ARESETN(m00_axis_aresetn),
        .M_AXIS_TVALID(m00_axis_tvalid),
        .M_AXIS_TDATA(m00_axis_tdata),
        .M_AXIS_TSTRB(m00_axis_tstrb),
        .M_AXIS_TLAST(m00_axis_tlast),
        .M_AXIS_TREADY(m00_axis_tready)
    );

// Instantiation of Axi Bus Interface S_AXI_INTR
    quad_adc_v1_0_S_AXI_INTR # (
        .C_S_AXI_DATA_WIDTH(C_S_AXI_INTR_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_INTR_ADDR_WIDTH),
        .C_NUM_OF_INTR(C_NUM_OF_INTR),
        .C_INTR_SENSITIVITY(C_INTR_SENSITIVITY),
        .C_INTR_ACTIVE_STATE(C_INTR_ACTIVE_STATE),
        .C_IRQ_SENSITIVITY(C_IRQ_SENSITIVITY),
        .C_IRQ_ACTIVE_STATE(C_IRQ_ACTIVE_STATE)
    ) quad_adc_v1_0_S_AXI_INTR_inst (
        .S_AXI_ACLK(s_axi_intr_aclk),
        .S_AXI_ARESETN(s_axi_intr_aresetn),
        .S_AXI_AWADDR(s_axi_intr_awaddr),
        .S_AXI_AWPROT(s_axi_intr_awprot),
        .S_AXI_AWVALID(s_axi_intr_awvalid),
        .S_AXI_AWREADY(s_axi_intr_awready),
        .S_AXI_WDATA(s_axi_intr_wdata),
        .S_AXI_WSTRB(s_axi_intr_wstrb),
        .S_AXI_WVALID(s_axi_intr_wvalid),
        .S_AXI_WREADY(s_axi_intr_wready),
        .S_AXI_BRESP(s_axi_intr_bresp),
        .S_AXI_BVALID(s_axi_intr_bvalid),
        .S_AXI_BREADY(s_axi_intr_bready),
        .S_AXI_ARADDR(s_axi_intr_araddr),
        .S_AXI_ARPROT(s_axi_intr_arprot),
        .S_AXI_ARVALID(s_axi_intr_arvalid),
        .S_AXI_ARREADY(s_axi_intr_arready),
        .S_AXI_RDATA(s_axi_intr_rdata),
        .S_AXI_RRESP(s_axi_intr_rresp),
        .S_AXI_RVALID(s_axi_intr_rvalid),
        .S_AXI_RREADY(s_axi_intr_rready),
        .irq(irq)
    );

    // Add user logic here

    // ADC Interfaces
    quad_adc_interface adc_ch_1 (
        .DATA_CLK(DATA_CLK),
        .FRAME_CLK(FRAME_CLK),
        .CH_X_A(CH_1_A),
        .CH_X_B(CH_1_B),
        .CH_X_DATA(ADC_CH_1_DATA)
    );
    quad_adc_interface adc_ch_2 (
        .DATA_CLK(DATA_CLK),
        .FRAME_CLK(FRAME_CLK),
        .CH_X_A(CH_2_A),
        .CH_X_B(CH_2_B),
        .CH_X_DATA(ADC_CH_2_DATA)
    );
    quad_adc_interface adc_ch_3 (
        .DATA_CLK(DATA_CLK),
        .FRAME_CLK(FRAME_CLK),
        .CH_X_A(CH_3_A),
        .CH_X_B(CH_3_B),
        .CH_X_DATA(ADC_CH_3_DATA)
    );
    quad_adc_interface adc_ch_4 (
        .DATA_CLK(DATA_CLK),
        .FRAME_CLK(FRAME_CLK),
        .CH_X_A(CH_4_A),
        .CH_X_B(CH_4_B),
        .CH_X_DATA(ADC_CH_4_DATA)
    );

    // Clock Domain Crossers
    clock_domain_crosser clock_domain_crosser_inst (
        .RESET_N(m00_axis_aresetn),
        .DATA_CLK(DATA_CLK),
        .FRAME_CLK(FRAME_CLK),
        .AXI_CLK(m00_axis_aclk),
        .AXI_DATA_VALID(AXI_DATA_VALID),

        .ADC_CH_1_DATA(ADC_CH_1_DATA),
        .ADC_CH_2_DATA(ADC_CH_2_DATA),
        .ADC_CH_3_DATA(ADC_CH_3_DATA),
        .ADC_CH_4_DATA(ADC_CH_4_DATA),

        .AXI_CH_1_DATA(AXI_CH_1_DATA),
        .AXI_CH_2_DATA(AXI_CH_2_DATA),
        .AXI_CH_3_DATA(AXI_CH_3_DATA),
        .AXI_CH_4_DATA(AXI_CH_4_DATA)
    );

    // Encode CLK Generator
    adc_encode_clk_gen adc_encode_clk_gen_inst (
        .AXI_CLK(m00_axis_aclk),
        .RESET_N(m00_axis_aresetn),
        .CLOCK_DIV(ENCODE_CLK_DIV),
        .ENCODE_CLK(ENCODE_CLK)
    );


    // User logic ends

    endmodule

`timescale 1 ns / 1 ps
module tb_clock_domain_crosser;

reg RESET_N;
reg DATA_CLK;
reg FRAME_CLK;
reg [13:0] ADC_CH_1_DATA;
reg [13:0] ADC_CH_2_DATA;
reg [13:0] ADC_CH_3_DATA;
reg [13:0] ADC_CH_4_DATA;
reg AXI_CLK;
wire AXI_DATA_VALID;
wire [13:0] AXI_CH_1_DATA;
wire [13:0] AXI_CH_2_DATA;
wire [13:0] AXI_CH_3_DATA;
wire [13:0] AXI_CH_4_DATA;

clock_domain_crosser uut (
    .RESET_N(RESET_N),
    .DATA_CLK(DATA_CLK),
    .FRAME_CLK(FRAME_CLK),
    .AXI_CLK(AXI_CLK),
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

parameter AXI_PERIOD = 10;
parameter FRAME_PERIOD = 96;
parameter DATA_PERIOD = 24;


// axi clock generation
initial begin
    AXI_CLK = 1'b0;
    #(AXI_PERIOD/2);
    forever
        #(AXI_PERIOD/2) AXI_CLK = ~AXI_CLK;
end

// frame clock generation
initial begin
    FRAME_CLK = 1'b0;
    #(FRAME_PERIOD/2);
    forever
        #(FRAME_PERIOD/2) FRAME_CLK = ~FRAME_CLK;
end

// axi clock generation
initial begin
    DATA_CLK = 1'b0;
    #(DATA_PERIOD/2 + DATA_PERIOD/4);
    forever
        #(DATA_PERIOD/2) DATA_CLK = ~DATA_CLK;
end

initial begin
    ADC_CH_1_DATA <= 14'hA00;
    ADC_CH_2_DATA <= 14'hB00;
    ADC_CH_3_DATA <= 14'hC00;
    ADC_CH_4_DATA <= 14'hD00;
    forever begin
        #(FRAME_PERIOD);
        ADC_CH_1_DATA <= ADC_CH_1_DATA  + 1;
        ADC_CH_2_DATA <= ADC_CH_2_DATA  + 1;
        ADC_CH_3_DATA <= ADC_CH_3_DATA  + 1;
        ADC_CH_4_DATA <= ADC_CH_4_DATA  + 1;
    end
end

initial begin
    RESET_N <= 0;
    #(AXI_PERIOD + DATA_PERIOD)

    RESET_N <= 1;
end

endmodule

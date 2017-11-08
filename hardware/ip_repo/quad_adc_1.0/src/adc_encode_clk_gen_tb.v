`timescale 1ns / 1ps
module tb_adc_encode_clk_gen();

reg AXI_CLK;
reg RESET_N;
reg [31:0] CLOCK_DIV;
wire ENCODE_CLK;

adc_encode_clk_gen uut (
    .AXI_CLK(AXI_CLK),
    .RESET_N(RESET_N),
    .CLOCK_DIV(CLOCK_DIV),
    .ENCODE_CLK(ENCODE_CLK)
);

parameter PERIOD = 10;

initial begin
    AXI_CLK <= 1'b0;
    #(PERIOD/2);
    forever
        #(PERIOD/2) AXI_CLK  = ~AXI_CLK;
end

initial begin
    RESET_N <= 0;
    CLOCK_DIV <= 10;
    #PERIOD
    RESET_N <= 1;
end

endmodule

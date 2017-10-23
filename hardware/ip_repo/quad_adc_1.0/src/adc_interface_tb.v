`timescale 1ns / 1ps
module TestBench();

    reg DATA_CLK;
    reg FRAME_CLK;
    reg CH_X_A;
    reg CH_X_B;
    wire [13 : 0] CH_X_DATA;

    //declare the unit under test
    quad_adc_interface  UUT(
        .DATA_CLK(DATA_CLK),
        .FRAME_CLK(FRAME_CLK),
        .CH_X_A(CH_X_A),
        .CH_X_B(CH_X_B),
        .CH_X_DATA(CH_X_DATA)
            );

// clock signal generation
initial begin
     DATA_CLK <= 0;
    forever #5 DATA_CLK = ~DATA_CLK;
end
initial begin
    FRAME_CLK <= 0;
    #3
    forever #20 FRAME_CLK = ~FRAME_CLK;
end


reg [13 : 0] OUT_DATA;
initial begin
    OUT_DATA <= 14'h2AAA;
end

initial begin
    CH_X_A <= 0;
    CH_X_B <= 0;
    #(10*6 + 4)
    for (integer i=0; i < 13; i=i+2) begin
        CH_X_A <= OUT_DATA[13-i];
        CH_X_B <= OUT_DATA[12-i];
        #5;
    end

    CH_X_A <= 1'b0;
    CH_X_B <= 1'b0;

    #5
    CH_X_A <= 1;
    CH_X_B <= 1;
end

endmodule

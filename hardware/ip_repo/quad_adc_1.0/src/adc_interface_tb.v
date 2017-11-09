`timescale 1ns / 1ps
module adc_interface_tb();

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
    forever #25 DATA_CLK = ~DATA_CLK;
end
initial begin
    FRAME_CLK <= 0;
    #12
    forever #100 FRAME_CLK = ~FRAME_CLK;
end


task shift_out_data;
    input [13 : 0] data;
begin
    for (integer i=0; i < 13; i=i+2) begin
        CH_X_A <= data[13-i];
        CH_X_B <= data[12-i];
        #25;
    end
    CH_X_A <= 1'b0;
    CH_X_B <= 1'b0;
    #25;
end
endtask

initial begin
    CH_X_A <= 0;
    CH_X_B <= 0;
    #(100*3 + 12)

    shift_out_data(14'h1);
    shift_out_data(14'h2);
    shift_out_data(14'h3);
    shift_out_data(14'h4);
    shift_out_data(14'h5);
    shift_out_data(14'h6);
    shift_out_data(14'h7);
end

endmodule


`timescale 1 ns / 1 ps

module adc_encode_clk_gen #
(

)
(
    input wire AXI_CLK,
    input wire RESET_N,
    input wire [31:0] CLOCK_DIV, // note: actual divider will by 2*CLOCK_DIV
    output wire ENCODE_CLK
);

    reg [31:0] counter;
    reg clk_track;

    always @(posedge AXI_CLK) begin
        if (!RESET_N) begin
            counter <= 0;
            clk_track <= 1'b0;
        end
        else if (counter == CLOCK_DIV - 1) begin
            counter <= 0;
            clk_track <= ~clk_track;
        end
        else begin
            counter <= counter + 1;
        end
    end

    assign ENCODE_CLK = clk_track;

endmodule

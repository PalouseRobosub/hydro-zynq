
`timescale 1 ns / 1 ps

	module quad_adc_interface #
	(
	)
	(
		// Users to add ports here
        input wire DATA_CLK,
        input wire FRAME_CLK,
        input wire CH_X_A,
        input wire CH_X_B,
        output wire [13 : 0] CH_X_DATA
    );

    reg [3 : 0] REG_CH_X_A_POS;
    reg [3 : 0] REG_CH_X_A_NEG;
    reg [3 : 0] REG_CH_X_B_POS;
    reg [3 : 0] REG_CH_X_B_NEG;

    reg [13 : 0] REG_CH_X_DATA;

    always @(posedge DATA_CLK) begin
        REG_CH_X_A_POS <= {REG_CH_X_A_POS[2:0],CH_X_A};
        REG_CH_X_B_POS <= {REG_CH_X_B_POS[2:0],CH_X_B};
    end

    always @(negedge DATA_CLK) begin
        REG_CH_X_A_NEG <= {REG_CH_X_A_NEG[2:0],CH_X_A};
        REG_CH_X_B_NEG <= {REG_CH_X_B_NEG[2:0],CH_X_B};
    end

    always @(posedge FRAME_CLK) begin
        REG_CH_X_DATA <= {REG_CH_X_A_POS[3],
                          REG_CH_X_B_POS[3],
                          REG_CH_X_A_NEG[3],
                          REG_CH_X_B_NEG[3],

                          REG_CH_X_A_POS[2],
                          REG_CH_X_B_POS[2],
                          REG_CH_X_A_NEG[2],
                          REG_CH_X_B_NEG[2],

                          REG_CH_X_A_POS[1],
                          REG_CH_X_B_POS[1],
                          REG_CH_X_A_NEG[1],
                          REG_CH_X_B_NEG[1],

                          REG_CH_X_A_POS[0],
                          REG_CH_X_B_POS[0]};
    end

    assign CH_X_DATA = REG_CH_X_DATA;

	endmodule

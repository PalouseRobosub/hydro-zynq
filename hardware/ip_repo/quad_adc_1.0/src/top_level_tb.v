`timescale 1 ns / 1 ps

module tb_quad_adc_v1_0;

parameter integer C_M_AXIS_TDATA_WIDTH = 32;

wire ENCODE_CLK;
reg  DATA_CLK;
reg  FRAME_CLK;
reg  CH_1_A, CH_1_B;
reg  CH_2_A, CH_2_B;
reg  CH_3_A, CH_3_B;
reg  CH_4_A, CH_4_B;
reg  m_axis_aclk;
reg  m_axis_aresetn;
wire m_axis_tvalid;
wire [C_M_AXIS_TDATA_WIDTH-1 : 0] m_axis_tdata;
wire m_axis_tlast;
reg  m_axis_tready;

quad_adc_v1_0 uut (
    .ENCODE_CLK             (    ENCODE_CLK             ),
    .DATA_CLK               (    DATA_CLK               ),
    .FRAME_CLK              (    FRAME_CLK              ),
    .CH_1_A                 (    CH_1_A                 ),
    .CH_2_A                 (    CH_2_A                 ),
    .CH_3_A                 (    CH_3_A                 ),
    .CH_4_A                 (    CH_4_A                 ),
    .CH_1_B                 (    CH_1_B                 ),
    .CH_2_B                 (    CH_2_B                 ),
    .CH_3_B                 (    CH_3_B                 ),
    .CH_4_B                 (    CH_4_B                 ),
    .m00_axis_aclk         (    m_axis_aclk         ),
    .s00_axi_aclk         (    m_axis_aclk         ),
    .m00_axis_aresetn      (    m_axis_aresetn      ),
    .s00_axi_aresetn      (    m_axis_aresetn      ),
    .m00_axis_tvalid       (    m_axis_tvalid       ),
    .m00_axis_tdata        (    m_axis_tdata        ),
    .m00_axis_tlast        (    m_axis_tlast        ),
    .m00_axis_tready       (    m_axis_tready       )
    );

// function for faking ADC
task shift_out_data;
    input [13 : 0] CH_1_DATA;
    input [13 : 0] CH_2_DATA;
    input [13 : 0] CH_3_DATA;
    input [13 : 0] CH_4_DATA;
begin
    for (integer i=0; i < 13; i=i+2) begin
        CH_1_A <= CH_1_DATA[13-i];
        CH_1_B <= CH_1_DATA[12-i];

        CH_2_A <= CH_2_DATA[13-i];
        CH_2_B <= CH_2_DATA[12-i];

        CH_3_A <= CH_3_DATA[13-i];
        CH_3_B <= CH_3_DATA[12-i];

        CH_4_A <= CH_4_DATA[13-i];
        CH_4_B <= CH_4_DATA[12-i];
        #25;
    end
    CH_1_A <= 1'b0;
    CH_1_B <= 1'b0;

    CH_2_A <= 1'b0;
    CH_2_B <= 1'b0;

    CH_3_A <= 1'b0;
    CH_3_B <= 1'b0;

    CH_4_A <= 1'b0;
    CH_4_B <= 1'b0;
    #25;
end
endtask

// clock signal generation
initial begin
     m_axis_aclk <= 0;
    forever #5 m_axis_aclk  = ~m_axis_aclk;
end
initial begin
     DATA_CLK <= 0;
    forever #25 DATA_CLK = ~DATA_CLK;
end
initial begin
    FRAME_CLK <= 0;
    #12
    forever #100 FRAME_CLK = ~FRAME_CLK;
end

// drive initial signals
initial begin
    CH_1_A <= 0;
    CH_2_A <= 0;
    CH_3_A <= 0;
    CH_4_A <= 0;


    CH_1_B <= 0;
    CH_2_B <= 0;
    CH_3_B <= 0;
    CH_4_B <= 0;

    m_axis_tready <= 0;
    m_axis_aresetn <= 0;

    #10
    m_axis_tready <= 1;
    m_axis_aresetn <= 1;

    #(100*3 + 2)


    shift_out_data(14'hA1, 14'hB1, 14'hC1, 14'hD1);
    shift_out_data(14'hA2, 14'hB2, 14'hC2, 14'hD2);
    shift_out_data(14'hA3, 14'hB3, 14'hC3, 14'hD3);
    shift_out_data(14'hA4, 14'hB4, 14'hC4, 14'hD4);

end


endmodule

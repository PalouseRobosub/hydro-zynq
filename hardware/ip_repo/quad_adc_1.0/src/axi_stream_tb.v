`timescale 1ns / 1ps
module axi_stream_tb();

    localparam integer C_M_AXIS_TDATA_WIDTH = 32;

    reg [15 : 0] ch_a_data_in;
    reg [15 : 0] ch_b_data_in;
    reg [15 : 0] ch_c_data_in;
    reg [15 : 0] ch_d_data_in;
    reg data_in_valid;
    reg  m_axis_aclk;
    reg  m_axis_aresetn;
    wire  m_axis_tvalid;
    wire [C_M_AXIS_TDATA_WIDTH-1 : 0] m_axis_tdata;
    wire [(C_M_AXIS_TDATA_WIDTH/8)-1 : 0] m_axis_tstrb;
    wire  m_axis_tlast;
    reg  m_axis_tready;

    quad_adc_v1_0_M00_AXIS  uut(
        .M_AXIS_ACLK(m_axis_aclk),
        .M_AXIS_ARESETN(m_axis_aresetn),
        .M_AXIS_TVALID(m_axis_tvalid),
        .M_AXIS_TDATA(m_axis_tdata),
        .M_AXIS_TSTRB(m_axis_tstrb),
        .M_AXIS_TLAST(m_axis_tlast),
        .M_AXIS_TREADY(m_axis_tready),

        .CH_A_DATA_IN(ch_a_data_in),
        .CH_B_DATA_IN(ch_b_data_in),
        .CH_C_DATA_IN(ch_c_data_in),
        .CH_D_DATA_IN(ch_d_data_in),
        .DATA_IN_VALID(data_in_valid)
    );


// clock generation
initial begin
    m_axis_aclk <= 0;
    forever #5 m_axis_aclk = ~m_axis_aclk;
end

// simulation
initial begin
    m_axis_tready <= 0;
    m_axis_aresetn <= 0;
    data_in_valid <= 1'b0;
    #10
    m_axis_aresetn <= 1;
    m_axis_tready <= 1;

    ch_a_data_in <= 16'hA;
    ch_b_data_in <= 16'hB;
    ch_c_data_in <= 16'hC;
    ch_d_data_in <= 16'hD;

    data_in_valid <= 1'b1;

    #10
    ch_a_data_in <= 16'hFA;
    ch_b_data_in <= 16'hFB;
    ch_c_data_in <= 16'hFC;
    ch_d_data_in <= 16'hFD;

    #25
    data_in_valid <= 1'b0;


    ch_a_data_in <= 16'hEA;
    ch_b_data_in <= 16'hEB;
    ch_c_data_in <= 16'hEC;
    ch_d_data_in <= 16'hED;

    #(10 * 5)
    data_in_valid <= 1'b1;
    #10
    data_in_valid <= 1'b0;

    end

endmodule

`timescale 1ns / 1ps
module TestBench();

    localparam integer C_M00_AXIS_TDATA_WIDTH = 32;

    reg  m00_axis_aclk;
	reg  m00_axis_aresetn;
    wire  m00_axis_tvalid;
    wire [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata;
    wire [(C_M00_AXIS_TDATA_WIDTH/8)-1 : 0] m00_axis_tstrb;
    wire  m00_axis_tlast;
    reg  m00_axis_tready;

    milliseconds_counter_v1_0 ms_counter(
        m00_axis_aclk,
        m00_axis_aresetn,
        m00_axis_tvalid,
        m00_axis_tdata,
        m00_axis_tstrb,
        m00_axis_tlast,
        m00_axis_tready
    );
    
    
initial begin
    m00_axis_aclk <= 0;
    forever #5 m00_axis_aclk = ~m00_axis_aclk;
end
initial begin
    m00_axis_tready <= 0;
    m00_axis_aresetn <= 0;
    #10
    m00_axis_aresetn <= 1;
    m00_axis_tready <= 1;
        #90

    //#900000
    m00_axis_tready <= 0;
    #20
    //#2000000
    m00_axis_tready <= 1;
    
    end
    
    
endmodule
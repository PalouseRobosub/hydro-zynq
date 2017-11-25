`timescale 1 ns / 1 ps

    module quad_adc_v1_0_M00_AXIS #
    (
        // Users to add parameters here

        // User parameters ends
        // Do not modify the parameters beyond this line

        // Width of S_AXIS address bus. The slave accepts the read and write addresses of width C_M_AXIS_TDATA_WIDTH.
        parameter integer C_M_AXIS_TDATA_WIDTH  = 32
    )
    (
        // Users to add ports here
        input wire [15 : 0] CH_A_DATA_IN,
        input wire [15 : 0] CH_B_DATA_IN,
        input wire [15 : 0] CH_C_DATA_IN,
        input wire [15 : 0] CH_D_DATA_IN,
        input wire FRAME_CLK,
        input wire [31 : 0] SAMPLES_PER_PACKET,

        // User ports ends
        // Do not modify the ports beyond this line

        // Global ports
        input wire  M_AXIS_ACLK,
        input wire  M_AXIS_ARESETN,
        output wire  M_AXIS_TVALID,
        output wire [C_M_AXIS_TDATA_WIDTH-1 : 0] M_AXIS_TDATA,
        output wire [(C_M_AXIS_TDATA_WIDTH/8)-1 : 0] M_AXIS_TSTRB,
        output wire  M_AXIS_TLAST,
        input wire  M_AXIS_TREADY
    );

    reg [15:0] CH_A_DATA_REG;
    reg [15:0] CH_B_DATA_REG;
    reg [15:0] CH_C_DATA_REG;
    reg [15:0] CH_D_DATA_REG;

    always @(posedge FRAME_CLK) begin
        CH_A_DATA_REG <= CH_A_DATA_IN;
        CH_D_DATA_REG <= CH_D_DATA_IN;
        CH_C_DATA_REG <= CH_C_DATA_IN;
        CH_B_DATA_REG <= CH_B_DATA_IN;
    end

    // hookup TDATA to its output
    //assign M_AXIS_TDATA  = stream_data_out;

    // TSTRB logic
    assign M_AXIS_TSTRB  = {(C_M_AXIS_TDATA_WIDTH/8){1'b1}};

    // Define the states of state machine
    // The control state machine oversees the writing of input streaming data to the FIFO,
    // and outputs the streaming data from the FIFO
    parameter [1:0] IDLE_STATE  = 2'b00, // This is the initial/idle state
                    TX_AB_STATE = 2'b01, // Transmitting CH_A and CH_B data
                    TX_CD_STATE = 2'b10, // Transmitting CH_C and CH_D data
                    WAIT_FOR_DATA_STATE = 2'b11; // Wait for new data

    wire [31:0] samples_per_packet = SAMPLES_PER_PACKET - 1;
   
    // State variable
    reg [1:0] state = IDLE_STATE;
    
    //Contains the number of samples sent in the current packet.
    reg [31:0] samples = 32'b0;

    // Control state machine implementation
    always @(posedge M_AXIS_ACLK)
    begin
      if (!M_AXIS_ARESETN)
      // Synchronous reset (active low)
        begin
          state <= IDLE_STATE;
          samples <= 32'b0;
        end
      else
        case (state)
          IDLE_STATE:
          begin
            if (FRAME_CLK == 1'b0) begin
                state <= WAIT_FOR_DATA_STATE;
            end
          end
          WAIT_FOR_DATA_STATE:
          begin
            if (FRAME_CLK == 1'b1) begin
                state  <= TX_AB_STATE;
            end
          end

          TX_AB_STATE:
          begin
            state <= TX_CD_STATE;
          end

          TX_CD_STATE:
          begin
            state <= IDLE_STATE;
            samples <= (samples >= samples_per_packet)? 32'b0 : samples + 1'b1;
          end

        endcase
    end


    // output drive logic
    //assign q = ( select == 0 )? d[0] : ( select == 1 )? d[1] : ( select == 2 )? d[2] : d[3];

    assign M_AXIS_TDATA =
        (state == IDLE_STATE)  ? {(C_M_AXIS_TDATA_WIDTH){1'b0}} :
        (state == TX_AB_STATE) ? {CH_B_DATA_REG,CH_A_DATA_REG} :
        (state == TX_CD_STATE) ? {CH_D_DATA_REG,CH_C_DATA_REG} :
        (state == WAIT_FOR_DATA_STATE) ? {(C_M_AXIS_TDATA_WIDTH){1'b0}} :
        {(C_M_AXIS_TDATA_WIDTH){1'b0}};
    /* always @ (*) begin */
    /*     case (state) */
    /*         IDLE_STATE: M_AXIS_TDATA = 32'hFABC; //{(C_M_AXIS_TDATA_WIDTH){1'b0}}; */
    /*         TX_AB_STATE: M_AXIS_TDATA = {CH_B_DATA_REG,CH_A_DATA_REG}; */
    /*         TX_CD_STATE: M_AXIS_TDATA = {CH_D_DATA_REG,CH_C_DATA_REG}; */
    /*     endcase */
    /* end */

    //tvalid generation
    //axis_tvalid is asserted when the control state machine's state is SEND_STREAM and
    //number of output streaming data is less than the NUMBER_OF_OUTPUT_WORDS.
    assign M_AXIS_TVALID = ((state == TX_AB_STATE) ||  (state == TX_CD_STATE));

    // AXI tlast generation
    // axis_tlast is asserted number of output streaming data is NUMBER_OF_OUTPUT_WORDS-1
    // (0 to NUMBER_OF_OUTPUT_WORDS-1)
    assign M_AXIS_TLAST = (samples == samples_per_packet) && (state == TX_CD_STATE);

    endmodule

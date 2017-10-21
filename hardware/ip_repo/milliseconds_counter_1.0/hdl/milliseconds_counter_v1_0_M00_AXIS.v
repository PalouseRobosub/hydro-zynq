
`timescale 1 ns / 1 ps

	module milliseconds_counter_v1_0_M00_AXIS #
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line

		// Width of S_AXIS address bus. The slave accepts the read and write addresses of width C_M_AXIS_TDATA_WIDTH.
		parameter integer C_M_AXIS_TDATA_WIDTH	= 32,
		// Start count is the numeber of clock cycles the master will wait before initiating/issuing any transaction.
		parameter integer C_M_START_COUNT	= 32
	)
	(
		// Users to add ports here

		// User ports ends
		// Do not modify the ports beyond this line

		// Global ports
		input wire  M_AXIS_ACLK,
		// 
		input wire  M_AXIS_ARESETN,
		// Master Stream Ports. TVALID indicates that the master is driving a valid transfer, A transfer takes place when both TVALID and TREADY are asserted. 
		output wire  M_AXIS_TVALID,
		// TDATA is the primary payload that is used to provide the data that is passing across the interface from the master.
		output wire [C_M_AXIS_TDATA_WIDTH-1 : 0] M_AXIS_TDATA,
		// TSTRB is the byte qualifier that indicates whether the content of the associated byte of TDATA is processed as a data byte or a position byte.
		output wire [(C_M_AXIS_TDATA_WIDTH/8)-1 : 0] M_AXIS_TSTRB,
		// TLAST indicates the boundary of a packet.
		output wire  M_AXIS_TLAST,
		// TREADY indicates that the slave can accept a transfer in the current cycle.
		input wire  M_AXIS_TREADY
	);
	//Total number of output data.
	// Total number of output data                                                 
//	localparam NUMBER_OF_OUTPUT_WORDS = 2;                                               
	                                                                                     
	// function called clogb2 that returns an integer which has the                      
	// value of the ceiling of the log base 2.                                           
//	function integer clogb2 (input integer bit_depth);                                   
//	  begin                                                                              
//	    for(clogb2=0; bit_depth>0; clogb2=clogb2+1)                                      
//	      bit_depth = bit_depth >> 1;                                                    
//	  end                                                                                
//	endfunction                                                                          
	                                                                                     
	// WAIT_COUNT_BITS is the width of the wait counter.                                 
	localparam integer WAIT_COUNT_BITS = 8;//clogb2(C_M_START_COUNT-1);                      
	                                                                                                            
	// Define the states of state machine                                                
	// The control state machine oversees the writing of input streaming data to the FIFO,
	// and outputs the streaming data from the FIFO                                      
	localparam [1:0] IDLE = 2'b00,        // This is the initial/idle state               
	                                                                                     
	                INIT_COUNTER  = 2'b01, // This state initializes the counter, ones   
	                                // the counter reaches C_M_START_COUNT count,        
	                                // the state machine changes state to INIT_WRITE     
	                SEND_STREAM   = 2'b10; // In this state the                          
	                                     // stream data is output through M_AXIS_TDATA   
	// State variable                                                                    
	reg [1:0] mst_exec_state;                                                                                                             

	// AXI Stream internal signals
	//wait counter. The master waits for the user defined number of clock cycles before initiating a transfer.
	reg [WAIT_COUNT_BITS-1 : 0] 	count;
	//streaming data valid
	wire  	axis_tvalid;

	//Last of the streaming data 
//	wire  	axis_tlast;

	//FIFO implementation signals
	reg [C_M_AXIS_TDATA_WIDTH-1 : 0] 	stream_data_out;


	// I/O Connections assignments

	assign M_AXIS_TVALID	= axis_tvalid;
	assign M_AXIS_TDATA	= stream_data_out;
	assign M_AXIS_TLAST	= 1'b1;
	assign M_AXIS_TSTRB	= {(C_M_AXIS_TDATA_WIDTH/8){1'b1}};


	// Control state machine implementation                             
	always @(posedge M_AXIS_ACLK)                                             
	begin                                                                     
	  if (!M_AXIS_ARESETN)                                                    
	  // Synchronous reset (active low)                                       
	    begin                                                                 
	      mst_exec_state <= IDLE;
	      stream_data_out <= 0;                                             
	      count    <= 0;                                                      
	    end                                                                   
	  else                                                                    
	    case (mst_exec_state)                                                 
	      IDLE:                                                               
	        // The slave starts accepting tdata when                          
	        // there tvalid is asserted to mark the                           
	        // presence of valid streaming data                               
	        //if ( count == 0 )                                                 
	        //  begin
	        if (new_data == 1) begin                                                          
	            mst_exec_state  <= INIT_COUNTER;
	            stream_data_out <= millisecond_counterR;
	            end                              
	        //  end                                                             
	        //else                                                              
	        //  begin                                                           
	        //    mst_exec_state  <= IDLE;                                      
	        //  end                                                             
	                                                                          
	      INIT_COUNTER:                                                       
	        // The slave starts accepting tdata when                          
	        // there tvalid is asserted to mark the                           
	        // presence of valid streaming data                               
	        if ( count == C_M_START_COUNT - 1 )                               
	          begin                                                           
	            mst_exec_state  <= SEND_STREAM;                               
	          end                                                             
	        else                                                              
	          begin                                                           
	            count <= count + 1;                                           
	            mst_exec_state  <= INIT_COUNTER;                              
	          end                                                             
	                                                                          
	      SEND_STREAM:                                                        
	        // The example design streaming master functionality starts       
	        // when the master drives output tdata from the FIFO and the slave
	        // has finished storing the S_AXIS_TDATA                          
	        if (axis_tvalid && M_AXIS_TREADY)                                                      
	          begin                                                           
	            mst_exec_state <= IDLE;                                       
	          end                                                             
	        else                                                              
	          begin                                                           
	            mst_exec_state <= SEND_STREAM;                                
	          end                                                             
	    endcase                                                               
	end                                                                       


	//tvalid generation
	//axis_tvalid is asserted when the control state machine's state is SEND_STREAM and
	//number of output streaming data is less than the NUMBER_OF_OUTPUT_WORDS.
	assign axis_tvalid = (mst_exec_state == SEND_STREAM);
	                                                                                               
	// AXI tlast generation                                                                        
	// axis_tlast is asserted number of output streaming data is NUMBER_OF_OUTPUT_WORDS-1          
	// (0 to NUMBER_OF_OUTPUT_WORDS-1)                                                             
//	assign axis_tlast = (mst_exec_state == SEND_STREAM);                              
	                                                                                               
	                                                                                               
	// Delay the axis_tvalid and axis_tlast signal by one clock cycle                              
	// to match the latency of M_AXIS_TDATA                                                        
//	always @(posedge M_AXIS_ACLK)                                                                  
//	begin                                                                                          
//	  if (!M_AXIS_ARESETN)                                                                         
//	    begin                                                                                      
//	      axis_tvalid_delay <= 1'b0;                                                               
//	      axis_tlast_delay <= 1'b0;                                                                
//	    end                                                                                        
//	  else                                                                                         
//	    begin                                                                                      
//	      axis_tvalid_delay <= axis_tvalid;                                                        
//	      axis_tlast_delay <= axis_tlast;                                                          
//	    end                                                                                        
//	end                                                                                                                                                                         
	                                                                                                

	// Add user logic here
	
	reg [                    31 : 0] sub_millisecond_counterR;
    reg [C_M_AXIS_TDATA_WIDTH-1 : 0] millisecond_counterR;
    reg                              new_data;

    
    
    //logic for incrementing our internal counters
    always @(posedge M_AXIS_ACLK) begin
       if ( ! M_AXIS_ARESETN ) begin
           sub_millisecond_counterR <= 1;
           millisecond_counterR <= 0;
           
           new_data <= 1;
           end
       else begin
           // assumming 100Mhz AXI clk, divide by 100,000 to get 1khz (1ms)
           if (sub_millisecond_counterR == 100000000) begin
               millisecond_counterR <= millisecond_counterR + 1;
               sub_millisecond_counterR <= 0;
               new_data <= 1;
               end
           else begin
               sub_millisecond_counterR <= sub_millisecond_counterR + 1;
               new_data <= 0;
               end
           end    
       end 

	// User logic ends

	endmodule

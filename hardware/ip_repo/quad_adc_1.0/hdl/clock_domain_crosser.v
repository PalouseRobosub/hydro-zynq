
`timescale 1 ns / 1 ps

    module clock_domain_crosser #
    (
    )
    (
        // Users to add ports here
        input RESET_N,
        input wire DATA_CLK,
        input wire FRAME_CLK,
        input wire [13 : 0] ADC_CH_1_DATA,
        input wire [13 : 0] ADC_CH_2_DATA,
        input wire [13 : 0] ADC_CH_3_DATA,
        input wire [13 : 0] ADC_CH_4_DATA,

        input wire AXI_CLK,
        output wire AXI_DATA_VALID,
        output wire [13 : 0] AXI_CH_1_DATA,
        output wire [13 : 0] AXI_CH_2_DATA,
        output wire [13 : 0] AXI_CH_3_DATA,
        output wire [13 : 0] AXI_CH_4_DATA
    );

    reg [13 : 0] ADC_CH_1_DATA_REG;
    reg [13 : 0] AXI_CH_1_DATA_REG;
    reg [13 : 0] ADC_CH_2_DATA_REG;
    reg [13 : 0] AXI_CH_2_DATA_REG;
    reg [13 : 0] ADC_CH_3_DATA_REG;
    reg [13 : 0] AXI_CH_3_DATA_REG;
    reg [13 : 0] ADC_CH_4_DATA_REG;
    reg [13 : 0] AXI_CH_4_DATA_REG;
    reg adc_data_valid;
    reg data_read;



    // data_valid generation
    parameter [1:0] ADC_IDLE_STATE = 2'b00, // This is the initial/idle state
                    ADC_WAIT_FOR_FRAME_STATE = 2'b01,
                    ADC_WAIT_FOR_DATA_READ_STATE = 2'b11;

    reg [1:0] adc_state;
    always @(posedge DATA_CLK or negedge RESET_N) begin
        if(!RESET_N) begin
            adc_state <= ADC_IDLE_STATE;
            ADC_CH_1_DATA_REG <= 0;
            ADC_CH_2_DATA_REG <= 0;
            ADC_CH_3_DATA_REG <= 0;
            ADC_CH_4_DATA_REG <= 0;
            adc_data_valid  <= 0;
        end
        else
            case (adc_state)
                ADC_IDLE_STATE: begin
                    if (FRAME_CLK == 0)
                        adc_state <= ADC_WAIT_FOR_FRAME_STATE;
                end

                ADC_WAIT_FOR_FRAME_STATE: begin
                    if (FRAME_CLK == 1) begin
                        ADC_CH_1_DATA_REG <= ADC_CH_1_DATA;
                        ADC_CH_2_DATA_REG <= ADC_CH_2_DATA;
                        ADC_CH_3_DATA_REG <= ADC_CH_3_DATA;
                        ADC_CH_4_DATA_REG <= ADC_CH_4_DATA;
                        adc_data_valid <= 1;
                        adc_state <= ADC_WAIT_FOR_DATA_READ_STATE;
                    end
                end

                ADC_WAIT_FOR_DATA_READ_STATE: begin
                    if (data_read == 1) begin
                        adc_data_valid <= 0;
                        adc_state <= ADC_IDLE_STATE;
                    end
                end
            endcase
    end


    parameter [1:0] AXI_IDLE_STATE = 2'b00, // This is the initial/idle state
                    AXI_HANDSHAKE_STATE = 2'b01;
    reg [1:0] axi_state;
    reg axi_data_valid;
    always @(posedge AXI_CLK) begin
        if(!RESET_N) begin
            axi_state <= AXI_IDLE_STATE;
            AXI_CH_1_DATA_REG <= 0;
            AXI_CH_2_DATA_REG <= 0;
            AXI_CH_3_DATA_REG <= 0;
            AXI_CH_4_DATA_REG <= 0;
            axi_data_valid <= 0;
            data_read <= 0;
        end
        else
            case(axi_state)
                AXI_IDLE_STATE: begin
                    if (adc_data_valid == 1) begin
                        AXI_CH_1_DATA_REG <= ADC_CH_1_DATA_REG;
                        AXI_CH_2_DATA_REG <= ADC_CH_2_DATA_REG;
                        AXI_CH_3_DATA_REG <= ADC_CH_3_DATA_REG;
                        AXI_CH_4_DATA_REG <= ADC_CH_4_DATA_REG;
                        data_read <= 1;
                        axi_data_valid <= 1;
                        axi_state <= AXI_HANDSHAKE_STATE;
                    end
                end

                AXI_HANDSHAKE_STATE: begin
                    axi_data_valid <= 0;
                    if (adc_data_valid == 0) begin
                        data_read <= 0;
                        axi_state <= AXI_IDLE_STATE;
                    end
                end
            endcase
    end

    assign AXI_CH_1_DATA = AXI_CH_1_DATA_REG;
    assign AXI_CH_2_DATA = AXI_CH_2_DATA_REG;
    assign AXI_CH_3_DATA = AXI_CH_3_DATA_REG;
    assign AXI_CH_4_DATA = AXI_CH_4_DATA_REG;
    assign AXI_DATA_VALID = axi_data_valid;

    endmodule

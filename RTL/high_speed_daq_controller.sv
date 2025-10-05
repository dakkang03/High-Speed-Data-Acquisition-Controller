module high_speed_daq_controller #(
    parameter NUM_CHANNELS = 8,
    parameter ADC_WIDTH = 12,
    parameter FIFO_DEPTH = 672,
    parameter CHANNEL_WIDTH = $clog2(NUM_CHANNELS),
    parameter TIMESTAMP_WIDTH = 32
)(
    input logic clk,
    input logic rst_n,
    
    input logic spi_sclk,
    input logic spi_mosi,
    output logic spi_miso,
    input logic spi_cs_n,
    
    output logic adc_start_conv,
    output logic [CHANNEL_WIDTH-1:0] adc_channel_sel,
    input logic adc_conv_done,
    input logic [ADC_WIDTH-1:0] adc_data,
    input logic adc_busy,
    
    output logic serial_data,
    output logic serial_clk,
    output logic serial_valid,
    
    output logic interrupt,
    output logic [7:0] status_leds
);

// =============================================================================
// Signal Declarations
// =============================================================================
logic [15:0] reg_addr_cdc;
logic [31:0] reg_wdata_cdc;
logic reg_write;
logic reg_write_pulse;
logic [3:0] write_hold_counter;

logic [31:0] config_registers [32-1:0];

typedef enum logic [2:0] {
    SPI_IDLE,
    SPI_CMD,
    SPI_ADDR_HIGH,
    SPI_ADDR_LOW,
    SPI_DATA_BYTE0,
    SPI_DATA_BYTE1,
    SPI_DATA_BYTE2,
    SPI_DATA_BYTE3
} spi_state_t;

spi_state_t spi_state, spi_next_state;
logic [2:0] bit_counter;
logic [7:0] spi_shift_reg;
logic spi_cs_n_sync, spi_cs_n_meta;

// SPI-domain storage for addr/data
logic [15:0] reg_addr_spi;
logic [31:0] reg_wdata_spi;

// Ready/Valid handshake signals
logic req_valid_spi;
logic ack_spi_sync1, ack_spi_sync2;
logic write_pending;     // Indicates write data is ready
logic spi_cs_n_sync_prev; // For edge detection

// CLK domain: sync req_valid and generate ack
logic req_valid_clk_sync1, req_valid_clk_sync2;
logic req_valid_clk_prev;
logic req_rising_edge_clk;
logic ack_clk;
logic ack_clk_sync1, ack_clk_sync2;

// Other signals
logic [CHANNEL_WIDTH-1:0] selected_channel;
logic channel_valid, channel_accept;
logic [NUM_CHANNELS-1:0] channel_enable, channel_ready, channel_urgent;
logic [1:0] arbiter_mode;
logic [3:0] channel_priority [NUM_CHANNELS-1:0];
logic [7:0] channel_weight [NUM_CHANNELS-1:0];

logic [15:0] fifo_wr_data;
logic fifo_wr_en, fifo_rd_en;
logic [15:0] fifo_rd_data;
logic fifo_full, fifo_empty;
logic [9:0] fifo_count;

logic trigger_detected, trigger_valid;
logic [7:0] trigger_confidence;
logic [15:0] trigger_metadata;
logic [31:0] trigger_config [8-1:0];

logic [31:0] throughput_sps, avg_latency_ns, max_latency_ns;
logic [7:0] fifo_utilization_pct;
logic [15:0] trigger_rate_ppm;
logic [7:0] warning_flags;
logic [31:0] debug_counters;

typedef enum logic [2:0] {
    IDLE,
    SELECT_CHANNEL,
    START_CONVERSION,
    WAIT_CONVERSION,
    CAPTURE_DATA,
    NEXT_CHANNEL
} adc_state_t;

adc_state_t current_state, next_state;
logic [7:0] settling_counter;
logic sample_ready;
logic [ADC_WIDTH-1:0] captured_data;
logic [TIMESTAMP_WIDTH-1:0] global_timestamp;

// =============================================================================
// SPI CS Synchronizer (spi_sclk domain)
// =============================================================================
always_ff @(posedge spi_sclk or negedge rst_n) begin
    if (!rst_n) begin
        spi_cs_n_meta <= 1'b1;
        spi_cs_n_sync <= 1'b1;
    end else begin
        spi_cs_n_meta <= spi_cs_n;
        spi_cs_n_sync <= spi_cs_n_meta;
    end
end

// =============================================================================
// SPI Slave Interface - PROPER CDC with CS edge detection
// Key fix: Trigger request on CS deassert after complete transaction
// =============================================================================
always_ff @(posedge spi_sclk or negedge rst_n) begin
    if (!rst_n) begin
        bit_counter <= 0;
        spi_shift_reg <= 0;
        spi_state <= SPI_IDLE;
        reg_addr_spi <= 0;
        reg_wdata_spi <= 0;
        req_valid_spi <= 1'b0;
        ack_spi_sync1 <= 1'b0;
        ack_spi_sync2 <= 1'b0;
        write_pending <= 1'b0;
        spi_cs_n_sync_prev <= 1'b1;
    end else begin
        // ALWAYS synchronize ack from CLK domain
        ack_spi_sync1 <= ack_clk_sync2;
        ack_spi_sync2 <= ack_spi_sync1;
        
        // Track CS for edge detection
        spi_cs_n_sync_prev <= spi_cs_n_sync;
        
        // Detect CS rising edge (deassert) - trigger pending write
        if (!spi_cs_n_sync_prev && spi_cs_n_sync) begin
            // CS just deasserted
            if (write_pending && !req_valid_spi) begin
                req_valid_spi <= 1'b1;
            end
            write_pending <= 1'b0;
        end
        
        if (spi_cs_n_sync) begin
            // CS deasserted - reset state but keep req_valid until ack
            bit_counter <= 0;
            spi_shift_reg <= 0;
            spi_state <= SPI_IDLE;
        end else begin
            // Shift in MOSI
            spi_shift_reg <= {spi_shift_reg[6:0], spi_mosi};
            if (bit_counter == 7) bit_counter <= 0; 
            else bit_counter <= bit_counter + 1;
            spi_state <= spi_next_state;

            // Capture data when byte complete
            if (bit_counter == 7) begin
                case (spi_state)
                    SPI_ADDR_HIGH:  reg_addr_spi[15:8]  <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_ADDR_LOW:   reg_addr_spi[7:0]   <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE0: reg_wdata_spi[31:24] <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE1: reg_wdata_spi[23:16] <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE2: reg_wdata_spi[15:8]  <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE3: begin
                        reg_wdata_spi[7:0] <= {spi_shift_reg[6:0], spi_mosi};
                        write_pending <= 1'b1;  // Mark write as pending
                    end
                    default: ;
                endcase
            end
        end
        
        // Clear request when ack received
        if (ack_spi_sync2 && req_valid_spi) begin
            req_valid_spi <= 1'b0;
        end
    end
end

// SPI state machine transitions
always_comb begin
    spi_next_state = spi_state;
    if (spi_cs_n_sync) spi_next_state = SPI_IDLE;
    else if (bit_counter == 7) begin
        case (spi_state)
            SPI_IDLE:       spi_next_state = SPI_CMD;
            SPI_CMD:        spi_next_state = SPI_ADDR_HIGH;
            SPI_ADDR_HIGH:  spi_next_state = SPI_ADDR_LOW;
            SPI_ADDR_LOW:   spi_next_state = SPI_DATA_BYTE0;
            SPI_DATA_BYTE0: spi_next_state = SPI_DATA_BYTE1;
            SPI_DATA_BYTE1: spi_next_state = SPI_DATA_BYTE2;
            SPI_DATA_BYTE2: spi_next_state = SPI_DATA_BYTE3;
            SPI_DATA_BYTE3: spi_next_state = SPI_IDLE;
            default:        spi_next_state = SPI_IDLE;
        endcase
    end
end

assign spi_miso = 1'b0;  // Read not implemented

// =============================================================================
// Request Synchronization: SPI domain -> CLK domain
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_valid_clk_sync1 <= 1'b0;
        req_valid_clk_sync2 <= 1'b0;
        req_valid_clk_prev <= 1'b0;
        req_rising_edge_clk <= 1'b0;
    end else begin
        // Two-stage synchronizer
        req_valid_clk_sync1 <= req_valid_spi;
        req_valid_clk_sync2 <= req_valid_clk_sync1;

        // Detect rising edge
        req_rising_edge_clk <= (req_valid_clk_sync2 && !req_valid_clk_prev);
        req_valid_clk_prev <= req_valid_clk_sync2;
    end
end

// =============================================================================
// Register Write Logic in CLK domain
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_addr_cdc <= 16'h0;
        reg_wdata_cdc <= 32'h0;
        reg_write_pulse <= 1'b0;
        write_hold_counter <= 4'd0;
        ack_clk <= 1'b0;
    end else begin
        if (req_rising_edge_clk) begin
            // Latch stable data from SPI domain
            reg_addr_cdc <= reg_addr_spi;
            reg_wdata_cdc <= reg_wdata_spi;
            reg_write_pulse <= 1'b1;
            write_hold_counter <= 4'd1;
        end else if (write_hold_counter > 0) begin
            write_hold_counter <= write_hold_counter - 1;
            reg_write_pulse <= (write_hold_counter > 1) ? 1'b1 : 1'b0;
        end else begin
            reg_write_pulse <= 1'b0;
        end

        // Assert ack AFTER write completes (one cycle delay)
        if (write_hold_counter == 1) begin
            ack_clk <= 1'b1;
        end else if (ack_clk) begin
            ack_clk <= 1'b0;  // Hold for only 1 cycle
        end
    end
end

assign reg_write = reg_write_pulse;

// =============================================================================
// Acknowledge Synchronization: CLK domain -> SPI domain
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ack_clk_sync1 <= 1'b0;
        ack_clk_sync2 <= 1'b0;
    end else begin
        ack_clk_sync1 <= ack_clk;
        ack_clk_sync2 <= ack_clk_sync1;
    end
end

// =============================================================================
// Configuration Registers
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < 32; i++) config_registers[i] <= 32'h0;
        config_registers[0] <= 32'h0000_0001;  // Enable
        config_registers[1] <= 32'h0000_FFFF;  // All channels
        config_registers[2] <= 32'h0000_0000;  // Round-robin mode
        for (int i = 0; i < 8; i++) begin
            config_registers[8+i] <= 32'h0000_0002;   // Default priority
            config_registers[16+i] <= 32'h0000_0001;  // Default weight
        end
    end else if (reg_write) begin
        automatic int reg_index;
        reg_index = reg_addr_cdc[7:2];
        if (reg_index < 32) begin
            config_registers[reg_index] <= reg_wdata_cdc;
        end
    end
end

assign channel_enable = config_registers[1][NUM_CHANNELS-1:0];
assign arbiter_mode = config_registers[2][1:0];

always_comb begin
    for (int i = 0; i < NUM_CHANNELS; i++) begin
        channel_priority[i] = config_registers[8+i][3:0];
        channel_weight[i] = config_registers[16+i][7:0];
    end

    for (int i = 0; i < 8; i++) begin
        trigger_config[i] = config_registers[4+i];
    end
end

// =============================================================================
// Serial Output Interface
// =============================================================================
logic [4:0] serial_bit_count;
logic [15:0] serial_shift_reg;
logic serial_active;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        serial_shift_reg <= 0;
        serial_bit_count <= 0;
        serial_active <= 0;
        fifo_rd_en <= 0;
    end else begin
        if (!serial_active && !fifo_empty) begin
            serial_shift_reg <= fifo_rd_data;
            serial_bit_count <= 0;
            serial_active <= 1;
            fifo_rd_en <= 1;
        end else if (serial_active) begin
            fifo_rd_en <= 0;
            serial_shift_reg <= {serial_shift_reg[14:0], 1'b0};
            serial_bit_count <= serial_bit_count + 1;
            
            if (serial_bit_count == 15) begin
                serial_active <= 0;
            end
        end else begin
            fifo_rd_en <= 0;
        end
    end
end

assign serial_data = serial_shift_reg[15];
assign serial_clk = clk && serial_active;
assign serial_valid = serial_active;

// =============================================================================
// Global Timestamp Counter
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) global_timestamp <= 0;
    else global_timestamp <= global_timestamp + 1;
end

// =============================================================================
// ADC Control State Machine
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
        settling_counter <= 0;
        captured_data <= 0;
    end else begin
        current_state <= next_state;
        if (current_state == SELECT_CHANNEL) settling_counter <= settling_counter + 1;
        else settling_counter <= 0;
        if (current_state == CAPTURE_DATA) captured_data <= adc_data;
    end
end

always_comb begin
    next_state = current_state;
    adc_start_conv = 1'b0;
    channel_accept = 1'b0;
    sample_ready = 1'b0;
    case (current_state)
        IDLE: 
            if (channel_valid && !adc_busy && config_registers[0][0]) 
                next_state = SELECT_CHANNEL;
        SELECT_CHANNEL: 
            if (settling_counter >= 8) 
                next_state = START_CONVERSION;
        START_CONVERSION: begin 
            adc_start_conv = 1'b1; 
            next_state = WAIT_CONVERSION; 
        end
        WAIT_CONVERSION: 
            if (adc_conv_done) 
                next_state = CAPTURE_DATA;
        CAPTURE_DATA: begin 
            sample_ready = 1'b1; 
            channel_accept = 1'b1; 
            next_state = NEXT_CHANNEL; 
        end
        NEXT_CHANNEL: 
            next_state = IDLE;
        default: 
            next_state = IDLE;
    endcase
end

assign channel_ready = channel_enable;
assign channel_urgent = '0;

// =============================================================================
// FIFO Write Logic
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_wr_en <= 1'b0;
        fifo_wr_data <= 16'h0;
    end else begin
        if (sample_ready && !fifo_full) begin
            fifo_wr_en <= 1'b1;
            fifo_wr_data <= {1'b0, selected_channel, captured_data};
        end else begin
            fifo_wr_en <= 1'b0;
        end
    end
end

// =============================================================================
// Status and Interrupt
// =============================================================================
assign status_leds = {
    fifo_full,
    fifo_empty,
    trigger_detected,
    warning_flags[4:0]
};

assign interrupt = trigger_detected || fifo_full || (|warning_flags);

// =============================================================================
// Module Instantiations
// =============================================================================

// Configurable Arbiter
configurable_arbiter #(
    .NUM_CHANNELS(NUM_CHANNELS)
) arbiter_inst (
    .clk(clk),
    .rst_n(rst_n),
    .arbiter_mode(arbiter_mode),
    .channel_priority(channel_priority),
    .channel_weight(channel_weight),
    .channel_enable(channel_enable),
    .channel_ready(channel_ready),
    .channel_urgent(channel_urgent),
    .adc_busy(adc_busy),
    .selected_channel(selected_channel),
    .channel_valid(channel_valid),
    .channel_accept(channel_accept)
);

// FIFO
single_fifo #(
    .DATA_WIDTH(16), 
    .FIFO_DEPTH(FIFO_DEPTH)
) fifo_inst (
    .clk(clk),
    .rst_n(rst_n),
    .fifo_mode(2'b00),
    .watermark_l1(8'd75),
    .watermark_l2(8'd80),
    .watermark_l3(8'd90),
    .wr_data(fifo_wr_data),
    .wr_en(fifo_wr_en),
    .wr_full(fifo_full),
    .wr_level(),
    .rd_data(fifo_rd_data),
    .rd_en(fifo_rd_en),
    .rd_empty(fifo_empty),
    .rd_level(),
    .fifo_status(),
    .level_overflow(),
    .backpressure_active(),
    .count(fifo_count)
);

// Trigger Engine
derivative_threshold_engine #(
    .NUM_CHANNELS(NUM_CHANNELS), 
    .ADC_WIDTH(ADC_WIDTH)
) trigger_inst (
    .clk(clk),
    .rst_n(rst_n),
    .data_in(captured_data),
    .channel_in(selected_channel),
    .data_valid(sample_ready),
    .config_reg(trigger_config),
    .trigger_out(trigger_detected),
    .trigger_confidence(trigger_confidence),
    .trigger_metadata(trigger_metadata),
    .trigger_valid(trigger_valid)
);

// Performance Monitor
performance_monitor #(
    .NUM_CHANNELS(NUM_CHANNELS)
) perf_mon (
    .clk(clk),
    .rst_n(rst_n),
    .sample_valid(sample_ready),
    .sample_channel(selected_channel),
    .sample_timestamp(global_timestamp),
    .fifo_wr_en(fifo_wr_en),
    .fifo_rd_en(fifo_rd_en),
    .fifo_count(fifo_count),
    .fifo_depth(10'd672),
    .fifo_full(fifo_full),
    .fifo_empty(fifo_empty),
    .trigger_detected(trigger_detected),
    .trigger_channel(selected_channel),
    .trigger_confidence(trigger_confidence),
    .adc_conversion_start(adc_start_conv),
    .adc_conversion_done(adc_conv_done),
    .adc_channel(selected_channel),
    .throughput_sps(throughput_sps),
    .avg_latency_ns(avg_latency_ns),
    .max_latency_ns(max_latency_ns),
    .fifo_utilization_pct(fifo_utilization_pct),
    .trigger_rate_ppm(trigger_rate_ppm),
    .warning_flags(warning_flags),
    .debug_counters(debug_counters)
);

assign adc_channel_sel = selected_channel;

endmodule

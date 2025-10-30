module derivative_threshold_engine #(
    parameter NUM_CHANNELS = 16,
    parameter ADC_WIDTH = 12,
    parameter CHANNEL_WIDTH = $clog2(NUM_CHANNELS),
    parameter CONFIG_REGS = 8
)(
    input logic clk,
    input logic rst_n,
    
    // Data input interface
    input logic [ADC_WIDTH-1:0] data_in,
    input logic [CHANNEL_WIDTH-1:0] channel_in,
    input logic data_valid,
    
    // Configuration registers (software configurable)
    input logic [31:0] config_reg [CONFIG_REGS-1:0],
    
    // Trigger output interface  
    output logic trigger_out,
    output logic [7:0] trigger_confidence,
    output logic [15:0] trigger_metadata,
    output logic trigger_valid
);

// Configuration register mapping
localparam CFG_THRESHOLD_LOW   = 0; // Basic amplitude threshold
localparam CFG_THRESHOLD_HIGH  = 1; // Derivative threshold  
localparam CFG_DERIVATIVE_EN   = 2; // Per-channel derivative enable mask
localparam CFG_CHANNEL_MASK    = 3; // Per-channel enable mask
localparam CFG_CONFIDENCE_MIN  = 4; // Minimum confidence threshold
localparam CFG_FILTER_WINDOW   = 5; // Anti-false-positive window
localparam CFG_DEBUG_CTRL      = 6; // Debug and test control
localparam CFG_STATUS          = 7; // Status register (read-only)

// Per-channel storage for derivative calculation
logic [ADC_WIDTH-1:0] prev_sample [NUM_CHANNELS-1:0];
logic [NUM_CHANNELS-1:0] sample_valid_history;

// Pipeline registers for timing
logic [ADC_WIDTH-1:0] data_reg;
logic [CHANNEL_WIDTH-1:0] channel_reg;
logic data_valid_reg;

// Derivative calculation signals
logic signed [ADC_WIDTH:0] derivative;      // 13-bit signed
logic [ADC_WIDTH-1:0] abs_derivative;       // 12-bit unsigned
logic derivative_overflow;

// Trigger detection signals  
logic threshold_trigger, derivative_trigger;
logic channel_enabled, derivative_enabled;
logic [7:0] calculated_confidence;
logic [15:0] calculated_metadata;

// Anti-false-positive filter
logic [3:0] trigger_history [NUM_CHANNELS-1:0]; // Last 4 triggers per channel
logic [7:0] recent_trigger_count;
logic false_positive_filter;

// Stage 1: Input registration and validation
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        data_reg <= '0;
        channel_reg <= '0;
        data_valid_reg <= 1'b0;
        sample_valid_history <= '0;
    end else begin
        data_reg <= data_in;
        channel_reg <= channel_in;
        data_valid_reg <= data_valid;
        
        // Track which channels have had at least one sample
        if (data_valid && channel_in < NUM_CHANNELS) begin
            sample_valid_history[channel_in] <= 1'b1;
        end
    end
end

// Stage 2: Derivative calculation  
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            prev_sample[i] <= '0;
        end
        derivative <= '0;
        derivative_overflow <= 1'b0;
    end else if (data_valid_reg && channel_reg < NUM_CHANNELS) begin
        // Calculate derivative (current - previous)
        derivative <= $signed({1'b0, data_reg}) - $signed({1'b0, prev_sample[channel_reg]});
        
        // Update previous sample for this channel
        prev_sample[channel_reg] <= data_reg;
        
        // Check for overflow in derivative calculation
        derivative_overflow <= (derivative > 2**ADC_WIDTH-1) || (derivative < -(2**ADC_WIDTH-1));
    end
end

// Absolute value of derivative
always_comb begin
    if (derivative < 0) begin
        abs_derivative = (-derivative > 2**ADC_WIDTH-1) ? 
                        (2**ADC_WIDTH-1) : -derivative[ADC_WIDTH-1:0];
    end else begin
        abs_derivative = (derivative > 2**ADC_WIDTH-1) ? 
                        (2**ADC_WIDTH-1) : derivative[ADC_WIDTH-1:0];
    end
end

// Stage 3: Configuration decode
always_comb begin
    channel_enabled = config_reg[CFG_CHANNEL_MASK][channel_reg] && 
                     sample_valid_history[channel_reg];
    
    derivative_enabled = config_reg[CFG_DERIVATIVE_EN][channel_reg];
end

// Stage 4: Trigger detection logic
always_comb begin
    // Basic amplitude threshold detection
    threshold_trigger = channel_enabled && 
                       (data_reg > config_reg[CFG_THRESHOLD_LOW][ADC_WIDTH-1:0]);
    
    // Derivative-based detection (rapid change)
    derivative_trigger = channel_enabled && 
                        derivative_enabled && 
                        (abs_derivative > config_reg[CFG_THRESHOLD_HIGH][ADC_WIDTH-1:0]);
end

// Stage 5: Confidence calculation
always_comb begin
    automatic logic [7:0] amplitude_factor, derivative_factor;
    
    // Amplitude contribution (0-127)
    amplitude_factor = data_reg[ADC_WIDTH-1:ADC_WIDTH-7];
    
    // Derivative contribution (0-127) 
    derivative_factor = abs_derivative[ADC_WIDTH-1:ADC_WIDTH-7];
    
    // Combined confidence with saturation
    if (amplitude_factor + derivative_factor > 255) begin
        calculated_confidence = 8'd255;
    end else begin
        calculated_confidence = amplitude_factor + derivative_factor;
    end
end

// Stage 6: Anti-false-positive filter
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            trigger_history[i] <= 4'b0000;
        end
        recent_trigger_count <= 8'b0;
    end else if (data_valid_reg && channel_reg < NUM_CHANNELS) begin
        // Shift history and add new trigger
        automatic logic new_trigger;
        new_trigger = threshold_trigger || derivative_trigger;
        trigger_history[channel_reg] <= {trigger_history[channel_reg][2:0], new_trigger};
        
        // Count recent triggers for this channel
        recent_trigger_count <= trigger_history[channel_reg][3] + 
                               trigger_history[channel_reg][2] +
                               trigger_history[channel_reg][1] + 
                               trigger_history[channel_reg][0] +
                               new_trigger;
    end
end

// False positive filtering logic
always_comb begin
    automatic logic [7:0] max_triggers_per_window;
    automatic logic [7:0] min_confidence;
    
    max_triggers_per_window = config_reg[CFG_FILTER_WINDOW][7:0];
    min_confidence = config_reg[CFG_CONFIDENCE_MIN][7:0];
    
    false_positive_filter = (recent_trigger_count <= max_triggers_per_window) &&
                           (calculated_confidence >= min_confidence);
end

// Stage 7: Metadata construction
always_comb begin
    calculated_metadata = {
        threshold_trigger,           // [15] Basic threshold triggered
        derivative_trigger,          // [14] Derivative threshold triggered  
        derivative_overflow,         // [13] Overflow occurred
        false_positive_filter,       // [12] Passed false positive filter
        channel_reg,                 // [11:8] Channel number
        calculated_confidence[7:0]   // [7:0] Confidence value
    };
end

// Final output stage
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        trigger_out <= 1'b0;
        trigger_confidence <= 8'b0;
        trigger_metadata <= 16'b0;
        trigger_valid <= 1'b0;
    end else begin
        automatic logic final_trigger;
        
        final_trigger = (threshold_trigger || derivative_trigger) && 
                       false_positive_filter && 
                       channel_enabled;
        
        trigger_out <= final_trigger;
        trigger_confidence <= calculated_confidence;
        trigger_metadata <= calculated_metadata;
        trigger_valid <= data_valid_reg && final_trigger;
    end
end

// Debug and status reporting (removed config_reg write - should be input only)
// Status register should be separate output if needed

// Simulation and debug support
`ifdef DEBUG
always_ff @(posedge clk) begin
    if (trigger_valid) begin
        $display("Time: %0t, TRIGGER: Ch%0d, Data=%0d, Deriv=%0d, Conf=%0d, Meta=0x%h", 
                 $time, channel_reg, data_reg, derivative, trigger_confidence, trigger_metadata);
    end
end
`endif

endmodule

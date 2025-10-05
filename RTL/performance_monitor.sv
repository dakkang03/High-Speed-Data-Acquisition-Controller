module performance_monitor #(
    parameter NUM_CHANNELS = 16,
    parameter CHANNEL_WIDTH = $clog2(NUM_CHANNELS),
    parameter COUNTER_WIDTH = 32,
    parameter TIMESTAMP_WIDTH = 32
)(
    input logic clk,
    input logic rst_n,
    
    // Data flow monitoring
    input logic sample_valid,
    input logic [CHANNEL_WIDTH-1:0] sample_channel,
    input logic [TIMESTAMP_WIDTH-1:0] sample_timestamp,
    
    // FIFO status monitoring
    input logic fifo_wr_en,
    input logic fifo_rd_en,
    input logic [9:0] fifo_count,
    input logic [9:0] fifo_depth,
    input logic fifo_full,
    input logic fifo_empty,
    
    // Trigger monitoring
    input logic trigger_detected,
    input logic [CHANNEL_WIDTH-1:0] trigger_channel,
    input logic [7:0] trigger_confidence,
    
    // ADC performance monitoring
    input logic adc_conversion_start,
    input logic adc_conversion_done,
    input logic [CHANNEL_WIDTH-1:0] adc_channel,
    
    // Performance metrics output
    output logic [31:0] throughput_sps,
    output logic [31:0] avg_latency_ns,
    output logic [31:0] max_latency_ns,
    output logic [7:0]  fifo_utilization_pct,
    output logic [15:0] trigger_rate_ppm,
    
    // Status and warnings
    output logic [7:0] warning_flags,
    output logic [31:0] debug_counters
);

// Warning flag definitions
localparam WARN_LOW_THROUGHPUT    = 0;
localparam WARN_HIGH_LATENCY      = 1; 
localparam WARN_HIGH_FIFO_USAGE   = 2;
localparam WARN_FIFO_OVERFLOW     = 3;
localparam WARN_HIGH_TRIGGER_RATE = 4;
localparam WARN_LOW_TRIGGER_RATE  = 5;
localparam WARN_ADC_TIMEOUT       = 6;
localparam WARN_SYSTEM_OVERLOAD   = 7;

// Throughput measurement
logic [31:0] throughput_counter;
logic [31:0] throughput_window_counter;
logic [31:0] timestamp_1sec_ago;
logic [31:0] samples_in_window;
logic [31:0] throughput_sps_reg;

// Latency measurement per channel
logic [TIMESTAMP_WIDTH-1:0] sample_start_time [NUM_CHANNELS-1:0];
logic [31:0] latency_sum [NUM_CHANNELS-1:0];
logic [31:0] latency_count [NUM_CHANNELS-1:0];
logic [31:0] latency_max [NUM_CHANNELS-1:0];
logic [31:0] current_latency;

// FIFO utilization tracking
logic [7:0] fifo_util_current;
logic [15:0] fifo_util_samples;
logic [31:0] fifo_util_sum;

// Trigger statistics
logic [31:0] trigger_count_total;
logic [31:0] sample_count_total;

// ADC performance tracking
logic [31:0] adc_conversion_time [NUM_CHANNELS-1:0];
logic [TIMESTAMP_WIDTH-1:0] adc_start_time [NUM_CHANNELS-1:0];
logic [31:0] adc_timeout_count;

localparam CLK_FREQ_HZ = 100_000_000;
localparam NS_PER_CYCLE = 1_000_000_000 / CLK_FREQ_HZ;

// Throughput measurement
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        throughput_counter <= 0;
        throughput_window_counter <= 0;
        samples_in_window <= 0;
        timestamp_1sec_ago <= 0;
        throughput_sps_reg <= 0;
    end else begin
        if (sample_valid) begin
            throughput_counter <= throughput_counter + 1;
        end
        
        if (sample_timestamp >= timestamp_1sec_ago + CLK_FREQ_HZ) begin
            throughput_sps_reg <= samples_in_window;
            samples_in_window <= 0;
            timestamp_1sec_ago <= sample_timestamp;
            throughput_window_counter <= throughput_window_counter + 1;
        end else if (sample_valid) begin
            samples_in_window <= samples_in_window + 1;
        end
    end
end

// Real-time throughput calculation for short tests
always_comb begin
    if (throughput_window_counter > 0) begin
        throughput_sps = throughput_sps_reg;
    end else if (sample_timestamp > 10000) begin
        // Instantaneous throughput after 10000 cycles
        throughput_sps = (throughput_counter * CLK_FREQ_HZ) / sample_timestamp;
    end else begin
        throughput_sps = 0;
    end
end

// Latency measurement
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            sample_start_time[i] <= 0;
            latency_sum[i] <= 0;
            latency_count[i] <= 0;
            latency_max[i] <= 0;
        end
        current_latency <= 0;
    end else begin
        if (sample_valid && sample_channel < NUM_CHANNELS) begin
            sample_start_time[sample_channel] <= sample_timestamp;
        end
        
        if (fifo_rd_en) begin
            automatic logic [CHANNEL_WIDTH-1:0] read_channel;
            read_channel = sample_channel;
            
            if (sample_start_time[read_channel] != 0) begin
                current_latency = (sample_timestamp - sample_start_time[read_channel]) * NS_PER_CYCLE;
                latency_sum[read_channel] <= latency_sum[read_channel] + current_latency;
                latency_count[read_channel] <= latency_count[read_channel] + 1;
                
                if (current_latency > latency_max[read_channel]) begin
                    latency_max[read_channel] <= current_latency;
                end
                
                sample_start_time[read_channel] <= 0;
            end
        end
    end
end

// Calculate average latency
logic [31:0] total_latency_sum, total_latency_count, global_max_latency;
always_comb begin
    total_latency_sum = 0;
    total_latency_count = 0;
    global_max_latency = 0;
    
    for (int i = 0; i < NUM_CHANNELS; i++) begin
        total_latency_sum = total_latency_sum + latency_sum[i];
        total_latency_count = total_latency_count + latency_count[i];
        if (latency_max[i] > global_max_latency) begin
            global_max_latency = latency_max[i];
        end
    end
    
    if (total_latency_count > 0) begin
        avg_latency_ns = total_latency_sum / total_latency_count;
    end else begin
        avg_latency_ns = 0;
    end
    
    max_latency_ns = global_max_latency;
end

// FIFO utilization tracking with overflow protection
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_util_sum <= 0;
        fifo_util_samples <= 0;
        fifo_util_current <= 0;
    end else begin
        if (fifo_depth > 0) begin
            fifo_util_current <= (fifo_count * 100) / fifo_depth;
        end else begin
            fifo_util_current <= 0;
        end
        
        // Limit accumulation to prevent overflow
        if (fifo_util_samples < 16'hFFFF) begin
            fifo_util_sum <= fifo_util_sum + fifo_util_current;
            fifo_util_samples <= fifo_util_samples + 1;
        end
    end
end

// FIFO utilization output with saturation
always_comb begin
    automatic logic [31:0] avg_temp;
    
    if (fifo_util_samples > 0) begin
        avg_temp = fifo_util_sum / fifo_util_samples;
        fifo_utilization_pct = (avg_temp > 100) ? 8'd100 : avg_temp[7:0];
    end else begin
        fifo_utilization_pct = fifo_util_current;
    end
end

// Trigger rate monitoring with real-time calculation
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        trigger_count_total <= 0;
        sample_count_total <= 0;
    end else begin
        if (trigger_detected) begin
            trigger_count_total <= trigger_count_total + 1;
        end
        
        if (sample_valid) begin
            sample_count_total <= sample_count_total + 1;
        end
    end
end

// Real-time trigger rate calculation
always_comb begin
    if (sample_count_total > 0) begin
        // triggers per million samples
        automatic logic [63:0] rate_calc;
        rate_calc = (trigger_count_total * 64'd1000000) / sample_count_total;
        trigger_rate_ppm = (rate_calc > 65535) ? 16'hFFFF : rate_calc[15:0];
    end else begin
        trigger_rate_ppm = 0;
    end
end

// ADC performance monitoring
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            adc_conversion_time[i] <= 0;
            adc_start_time[i] <= 0;
        end
        adc_timeout_count <= 0;
    end else begin
        if (adc_conversion_start && adc_channel < NUM_CHANNELS) begin
            adc_start_time[adc_channel] <= sample_timestamp;
        end
        
        if (adc_conversion_done && adc_channel < NUM_CHANNELS) begin
            if (adc_start_time[adc_channel] != 0) begin
                adc_conversion_time[adc_channel] <= sample_timestamp - adc_start_time[adc_channel];
                adc_start_time[adc_channel] <= 0;
            end
        end
        
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            if (adc_start_time[i] != 0 && 
                (sample_timestamp - adc_start_time[i]) > (CLK_FREQ_HZ / 1000)) begin
                adc_timeout_count <= adc_timeout_count + 1;
                adc_start_time[i] <= 0;
            end
        end
    end
end

// Warning flag generation with proper bit counting
always_comb begin
    automatic int flag_count;
    warning_flags = 8'b0;
    flag_count = 0;
    
    if (throughput_sps < 14400 && throughput_sps > 0) begin
        warning_flags[WARN_LOW_THROUGHPUT] = 1'b1;
        flag_count++;
    end
    
    if (avg_latency_ns > 1_000_000) begin
        warning_flags[WARN_HIGH_LATENCY] = 1'b1;
        flag_count++;
    end
    
    if (fifo_utilization_pct > 80) begin
        warning_flags[WARN_HIGH_FIFO_USAGE] = 1'b1;
        flag_count++;
    end
    
    if (fifo_full && fifo_wr_en) begin
        warning_flags[WARN_FIFO_OVERFLOW] = 1'b1;
        flag_count++;
    end
    
    if (trigger_rate_ppm > 100_000) begin
        warning_flags[WARN_HIGH_TRIGGER_RATE] = 1'b1;
        flag_count++;
    end else if (trigger_rate_ppm < 100 && sample_count_total > 1000) begin
        warning_flags[WARN_LOW_TRIGGER_RATE] = 1'b1;
        flag_count++;
    end
    
    if (adc_timeout_count > 0) begin
        warning_flags[WARN_ADC_TIMEOUT] = 1'b1;
        flag_count++;
    end
    
    if (flag_count >= 3) begin
        warning_flags[WARN_SYSTEM_OVERLOAD] = 1'b1;
    end
end

assign debug_counters = {
    throughput_counter[15:0],
    trigger_count_total[15:0]
};

`ifdef DEBUG
always_ff @(posedge clk) begin
    if (throughput_window_counter > 0 && throughput_window_counter % 10 == 0) begin
        $display("Time: %0t, PERF: Throughput=%0d SPS, Latency=%0d ns, FIFO=%0d%%, Triggers=%0d ppm", 
                 $time, throughput_sps, avg_latency_ns, fifo_utilization_pct, trigger_rate_ppm);
    end
    
    if (|warning_flags) begin
        $display("Time: %0t, WARNING: Flags=0x%02h", $time, warning_flags);
    end
end
`endif

endmodule

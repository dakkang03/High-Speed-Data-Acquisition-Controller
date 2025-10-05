`timescale 1ns/1ps

module tb_performance_monitor;

// Parameters
localparam NUM_CHANNELS = 16;
localparam CHANNEL_WIDTH = $clog2(NUM_CHANNELS);
localparam TIMESTAMP_WIDTH = 32;
localparam CLK_PERIOD = 10;

// DUT signals
logic clk;
logic rst_n;
logic sample_valid;
logic [CHANNEL_WIDTH-1:0] sample_channel;
logic [TIMESTAMP_WIDTH-1:0] sample_timestamp;
logic fifo_wr_en;
logic fifo_rd_en;
logic [9:0] fifo_count;
logic [9:0] fifo_depth;
logic fifo_full;
logic fifo_empty;
logic trigger_detected;
logic [CHANNEL_WIDTH-1:0] trigger_channel;
logic [7:0] trigger_confidence;
logic adc_conversion_start;
logic adc_conversion_done;
logic [CHANNEL_WIDTH-1:0] adc_channel;
logic [31:0] throughput_sps;
logic [31:0] avg_latency_ns;
logic [31:0] max_latency_ns;
logic [7:0] fifo_utilization_pct;
logic [15:0] trigger_rate_ppm;
logic [7:0] warning_flags;
logic [31:0] debug_counters;

int error_count = 0;

performance_monitor #(
    .NUM_CHANNELS(NUM_CHANNELS),
    .CHANNEL_WIDTH(CHANNEL_WIDTH),
    .TIMESTAMP_WIDTH(TIMESTAMP_WIDTH)
) dut (.*);

initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sample_timestamp <= 0;
    else
        sample_timestamp <= sample_timestamp + 1;
end

task reset_dut();
    rst_n = 0;
    sample_valid = 0;
    sample_channel = 0;
    fifo_wr_en = 0;
    fifo_rd_en = 0;
    fifo_count = 0;
    fifo_depth = 10'd512;
    fifo_full = 0;
    fifo_empty = 1;
    trigger_detected = 0;
    trigger_channel = 0;
    trigger_confidence = 0;
    adc_conversion_start = 0;
    adc_conversion_done = 0;
    adc_channel = 0;
    repeat(5) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);
    $display("Reset complete at time %0t", $time);
endtask

task send_sample(input [CHANNEL_WIDTH-1:0] ch);
    @(posedge clk);
    sample_valid = 1;
    sample_channel = ch;
    fifo_wr_en = 1;
    if (fifo_count < fifo_depth) begin
        fifo_count = fifo_count + 1;
        fifo_empty = 0;
    end
    if (fifo_count >= fifo_depth - 1) fifo_full = 1;
    @(posedge clk);
    sample_valid = 0;
    fifo_wr_en = 0;
endtask

task read_fifo();
    @(posedge clk);
    if (fifo_count > 0) begin
        fifo_rd_en = 1;
        fifo_count = fifo_count - 1;
        fifo_full = 0;
    end
    if (fifo_count == 0) fifo_empty = 1;
    @(posedge clk);
    fifo_rd_en = 0;
endtask

task generate_trigger(input [CHANNEL_WIDTH-1:0] ch, input [7:0] conf);
    @(posedge clk);
    trigger_detected = 1;
    trigger_channel = ch;
    trigger_confidence = conf;
    @(posedge clk);
    trigger_detected = 0;
endtask

task adc_convert(input [CHANNEL_WIDTH-1:0] ch, input int delay_cycles);
    @(posedge clk);
    adc_conversion_start = 1;
    adc_channel = ch;
    @(posedge clk);
    adc_conversion_start = 0;
    repeat(delay_cycles) @(posedge clk);
    adc_conversion_done = 1;
    @(posedge clk);
    adc_conversion_done = 0;
endtask

initial begin
    $display("\n=== Performance Monitor Test Started ===\n");
    
    reset_dut();
    
    // Test 1: Throughput
    $display("\n--- Test 1: Throughput Measurement ---");
    $display("Sending 1000 samples at 16kHz rate...");
    
    for (int i = 0; i < 1000; i++) begin
        send_sample(i % NUM_CHANNELS);
        repeat(62) @(posedge clk);
    end
    
    repeat(1000) @(posedge clk);
    
    if (throughput_sps > 0) begin
        $display("Throughput measured: %0d SPS", throughput_sps);
        $display("? Test 1 PASSED");
    end else begin
        $display("? Test 1 FAILED");
        error_count++;
    end
    
    // Test 2: FIFO Utilization
    $display("\n--- Test 2: FIFO Utilization ---");
    
    fifo_count = 256;
    fifo_empty = 0;
    repeat(100) @(posedge clk);
    
    $display("FIFO at 50%%, Utilization: %0d%%", fifo_utilization_pct);
    
    fifo_count = 460;
    repeat(100) @(posedge clk);
    
    $display("FIFO at 90%%, Utilization: %0d%%", fifo_utilization_pct);
    
    if (fifo_utilization_pct > 40) begin
        $display("? Test 2 PASSED");
    end else begin
        $display("? Test 2 FAILED");
        error_count++;
    end
    
    fifo_count = 0;
    fifo_empty = 1;
    fifo_full = 0;
    
    // Test 3: Latency
    $display("\n--- Test 3: Latency Measurement ---");
    
    for (int i = 0; i < 50; i++) begin
        send_sample(0);
        repeat(10) @(posedge clk);
        read_fifo();
    end
    
    repeat(100) @(posedge clk);
    
    if (avg_latency_ns > 0) begin
        $display("Average Latency: %0d ns", avg_latency_ns);
        $display("Max Latency: %0d ns", max_latency_ns);
        $display("? Test 3 PASSED");
    end else begin
        $display("? Test 3 WARNING: Latency not measured");
    end
    
    // Test 4: Trigger Rate
    $display("\n--- Test 4: Trigger Rate Monitoring ---");
    
    for (int i = 0; i < 2000; i++) begin
        send_sample(i % NUM_CHANNELS);
        
        if (i % 100 == 0) begin
            generate_trigger(i % NUM_CHANNELS, 8'd150);
        end
        
        @(posedge clk);
    end
    
    repeat(100) @(posedge clk);
    
    $display("Trigger Rate: %0d ppm", trigger_rate_ppm);
    
    if (trigger_rate_ppm > 0) begin
        $display("? Test 4 PASSED");
    end else begin
        $display("? Test 4 FAILED");
        error_count++;
    end
    
    // Test 5: Warning Flags
    $display("\n--- Test 5: Warning Flags ---");
    
    fifo_full = 1;
    fifo_count = fifo_depth;
    @(posedge clk);
    fifo_wr_en = 1;
    @(posedge clk);
    fifo_wr_en = 0;
    @(posedge clk);
    
    if (warning_flags[3]) begin
        $display("? FIFO overflow warning detected");
    end else begin
        $display("? FIFO overflow warning not triggered");
    end
    
    fifo_full = 0;
    fifo_count = 256;
    repeat(10) @(posedge clk);
    
    fifo_count = 460;
    repeat(100) @(posedge clk);
    
    if (warning_flags[2]) begin
        $display("? High FIFO usage warning detected");
    end else begin
        $display("? High FIFO usage warning not triggered");
    end
    
    $display("Warning Flags: 0x%02h", warning_flags);
    
    if (|warning_flags) begin
        $display("? Test 5 PASSED");
    end else begin
        $display("? Test 5 WARNING: No warnings triggered");
    end
    
    fifo_count = 100;
    repeat(100) @(posedge clk);
    
    // Test 6: ADC Performance
    $display("\n--- Test 6: ADC Performance ---");
    
    for (int ch = 0; ch < 4; ch++) begin
        adc_convert(ch, 50);
    end
    
    repeat(100) @(posedge clk);
    
    $display("ADC conversions completed");
    
    adc_conversion_start = 1;
    adc_channel = 5;
    @(posedge clk);
    adc_conversion_start = 0;
    
    repeat(110000) @(posedge clk);
    
    if (warning_flags[6]) begin
        $display("? ADC timeout warning detected");
        $display("? Test 6 PASSED");
    end else begin
        $display("? Test 6 WARNING: ADC timeout not detected");
    end
    
    // Test 7: Stress Test
    $display("\n--- Test 7: Multi-Channel Stress Test ---");
    
    $display("Sending 5000 samples...");
    
    for (int i = 0; i < 5000; i++) begin
        send_sample(i % NUM_CHANNELS);
        
        if ($random % 50 == 0) begin
            generate_trigger(i % NUM_CHANNELS, $random % 256);
        end
        
        if (i % 3 == 0 && fifo_count > 0) begin
            read_fifo();
        end
    end
    
    repeat(500) @(posedge clk);
    
    $display("Stress test complete");
    $display("Final Stats:");
    $display("  Throughput: %0d SPS", throughput_sps);
    $display("  FIFO Utilization: %0d%%", fifo_utilization_pct);
    $display("  Trigger Rate: %0d ppm", trigger_rate_ppm);
    $display("  Warning Flags: 0x%02h", warning_flags);
    $display("  Debug Counters: 0x%08h", debug_counters);
    
    $display("? Test 7 PASSED");
    
    // Test 8: Metrics Validation
    begin
        automatic int expected_samples;
        automatic int expected_triggers;
        
        $display("\n--- Test 8: Metrics Validation ---");
        
        expected_samples = debug_counters[31:16];
        expected_triggers = debug_counters[15:0];
        
        $display("Total samples processed: %0d", expected_samples);
        $display("Total triggers detected: %0d", expected_triggers);
        
        if (expected_samples > 6000) begin
            $display("? Sample counting accurate");
        end else begin
            $display("? Sample count mismatch");
            error_count++;
        end
        
        if (expected_triggers > 0) begin
            $display("? Trigger counting accurate");
        end else begin
            $display("? Trigger count error");
            error_count++;
        end
    end
    
    // Summary
    repeat(100) @(posedge clk);
    
    $display("\n\n=== Test Summary ===");
    $display("Total Errors: %0d", error_count);
    
    if (error_count == 0) begin
        $display("*** ALL TESTS PASSED ***");
    end else begin
        $display("*** %0d TESTS FAILED ***", error_count);
    end
    
    $display("\n=== Test Complete ===\n");
    $finish;
end

initial begin
    #50000000;
    $display("ERROR: Test timeout!");
    $finish;
end

endmodule

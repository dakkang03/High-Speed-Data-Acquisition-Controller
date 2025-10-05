`timescale 1ns/1ps

module tb_trigger;

logic clk, rst_n;
logic [11:0] data_in;
logic [3:0] channel_in;
logic data_valid;
logic [31:0] config_reg [7:0];
logic trigger_out;
logic [7:0] trigger_confidence;
logic [15:0] trigger_metadata;
logic trigger_valid;

derivative_threshold_engine #(
    .NUM_CHANNELS(16),
    .ADC_WIDTH(12)
) dut (.*);

initial clk = 0;
always #5 clk = ~clk;

initial begin
    rst_n = 0;
    data_in = 0;
    channel_in = 0;
    data_valid = 0;
    
    // Config: thresholds and masks
    config_reg[0] = 32'h0000_0800;  // Threshold low: 2048
    config_reg[1] = 32'h0000_0400;  // Derivative threshold: 1024
    config_reg[2] = 32'h0000_FFFF;  // Derivative enable all
    config_reg[3] = 32'h0000_FFFF;  // Channel mask all
    config_reg[4] = 32'h0000_0080;  // Min confidence: 128
    config_reg[5] = 32'h0000_0003;  // Filter window: 3
    config_reg[6] = 32'h0000_0000;  // Debug
    config_reg[7] = 32'h0000_0000;  // Status
    
    repeat(10) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);
    
    // Test 1: Basic threshold trigger
    $display("=== Test 1: Amplitude Threshold ===");
    for (int i = 0; i < 10; i++) begin
        @(posedge clk);
        data_valid = 1;
        channel_in = 0;
        data_in = 100 + i * 200;  // Increasing amplitude
        if (trigger_out) $display("Trigger at data=%0d", data_in);
    end
    data_valid = 0;
    repeat(5) @(posedge clk);
    
    // Test 2: Derivative detection (rapid change)
    $display("\n=== Test 2: Derivative Trigger ===");
    channel_in = 1;
    @(posedge clk);
    data_valid = 1;
    data_in = 500;
    @(posedge clk);
    data_in = 500;  // No change
    @(posedge clk);
    data_in = 2000;  // Large jump
    @(posedge clk);
    if (trigger_out) $display("Derivative trigger: confidence=%0d", trigger_confidence);
    data_valid = 0;
    repeat(5) @(posedge clk);
    
    // Test 3: Multi-channel with history
    $display("\n=== Test 3: Multi-Channel History ===");
    for (int ch = 0; ch < 4; ch++) begin
        for (int sample = 0; sample < 5; sample++) begin
            @(posedge clk);
            data_valid = 1;
            channel_in = ch;
            data_in = 1000 + ch * 100 + sample * 50;
        end
    end
    data_valid = 0;
    
    // Test 4: False positive filter
    $display("\n=== Test 4: False Positive Filter ===");
    config_reg[5] = 32'h0000_0001;  // Max 1 trigger per window
    channel_in = 2;
    
    for (int i = 0; i < 10; i++) begin
        @(posedge clk);
        data_valid = 1;
        data_in = 3000;  // Always above threshold
        if (trigger_out) $display("Trigger %0d (should be filtered)", i);
    end
    data_valid = 0;
    
    // Test 5: Derivative overflow
    $display("\n=== Test 5: Derivative Overflow ===");
    channel_in = 3;
    @(posedge clk);
    data_valid = 1;
    data_in = 12'h000;  // Min
    @(posedge clk);
    data_in = 12'hFFF;  // Max (huge derivative)
    @(posedge clk);
    $display("Overflow handling: meta=%04h", trigger_metadata);
    data_valid = 0;
    
    // Test 6: Confidence calculation edge cases
    $display("\n=== Test 6: Confidence Edges ===");
    channel_in = 4;
    
    // Zero derivative
    @(posedge clk);
    data_valid = 1;
    data_in = 2000;
    @(posedge clk);
    data_in = 2000;  // No change
    @(posedge clk);
    $display("Zero derivative confidence: %0d", trigger_confidence);
    
    // Max values
    @(posedge clk);
    data_in = 12'hFFF;
    @(posedge clk);
    data_in = 12'hFFF;
    @(posedge clk);
    $display("Max value confidence: %0d", trigger_confidence);
    
    data_valid = 0;
    
    // Test 7: Channel enable/disable
    $display("\n=== Test 7: Channel Masking ===");
    config_reg[3] = 32'h0000_000F;  // Only Ch 0-3
    
    for (int ch = 0; ch < 8; ch++) begin
        @(posedge clk);
        data_valid = 1;
        channel_in = ch;
        data_in = 3000;  // Above threshold
        @(posedge clk);
        if (trigger_out) $display("Ch%0d triggered", ch);
        else $display("Ch%0d masked", ch);
    end
    data_valid = 0;
    
    $display("\n=== Trigger Tests Complete ===");
    $finish;
end

endmodule

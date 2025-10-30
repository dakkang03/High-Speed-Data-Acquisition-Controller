// =============================================================================
// File: tb_arbiter.sv
// Description: Standalone arbiter testbench for maximum coverage
// =============================================================================

`timescale 1ns/1ps

module tb_arbiter;

logic clk, rst_n;
logic [1:0] arbiter_mode;
logic [3:0] channel_priority [15:0];
logic [7:0] channel_weight [15:0];
logic [15:0] channel_enable, channel_ready, channel_urgent;
logic adc_busy;
logic [3:0] selected_channel;
logic channel_valid, channel_accept;

// DUT
configurable_arbiter #(.NUM_CHANNELS(16)) dut (
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

// Clock generation
initial clk = 0;
always #5 clk = ~clk;

// Main test
initial begin
    // Initialize
    rst_n = 0;
    adc_busy = 0;
    channel_accept = 0;
    arbiter_mode = 0;
    channel_enable = 16'hFFFF;
    channel_ready = 16'h0000;
    channel_urgent = 0;
    
    for (int i = 0; i < 16; i++) begin
        channel_priority[i] = 0;
        channel_weight[i] = 1;
    end
    
    repeat(10) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);
    
    // =========================================================================
    // Test 1: Mode 0 - Round-Robin
    // =========================================================================
    $display("\n=== Test 1: Mode 0 Round-Robin ===");
    arbiter_mode = 2'b00;
    channel_ready = 16'h000F;
    
    for (int i = 0; i < 8; i++) begin
        @(posedge clk);
        $display("Cycle %0d: Ch%0d (valid=%0b)", i, selected_channel, channel_valid);
    end
    
    // =========================================================================
    // Test 2: Mode 1 - Priority with Equal Values (Tie-breaking)
    // =========================================================================
    $display("\n=== Test 2: Mode 1 Equal Priority (Tie-breaking) ===");
    arbiter_mode = 2'b01;
    channel_priority[0] = 5;
    channel_priority[1] = 5;
    channel_priority[2] = 5;
    channel_priority[3] = 5;
    channel_ready = 16'h000F;
    
    repeat(10) @(posedge clk);
    $display("Selected: Ch%0d (first with priority 5)", selected_channel);
    
    // =========================================================================
    // Test 3: Mode 1 - Priority Comparison Coverage
    // =========================================================================
    $display("\n=== Test 3: Mode 1 Priority Comparisons ===");
    channel_enable = 16'hFFFF;
    channel_ready = 16'h00FF;  // Ch 0-7
    
    // Set various priorities to trigger comparisons
    channel_priority[0] = 1;
    channel_priority[1] = 3;
    channel_priority[2] = 5;
    channel_priority[3] = 7;
    channel_priority[4] = 9;
    channel_priority[5] = 2;
    channel_priority[6] = 4;
    channel_priority[7] = 6;
    
    for (int cycle = 0; cycle < 30; cycle++) begin
        @(posedge clk);
        
        // Change priorities to trigger different comparisons
        if (cycle == 5) begin
            channel_priority[0] = 15;  // Make Ch0 highest
            $display("  Cycle %0d: Changed Ch0 priority to 15", cycle);
        end
        if (cycle == 10) begin
            channel_priority[7] = 15;  // Make Ch7 highest
            $display("  Cycle %0d: Changed Ch7 priority to 15", cycle);
        end
        if (cycle == 15) begin
            channel_priority[3] = 15;  // Make Ch3 highest
            $display("  Cycle %0d: Changed Ch3 priority to 15", cycle);
        end
        
        if (cycle % 5 == 0) begin
            $display("  Cycle %0d: Selected Ch%0d", cycle, selected_channel);
        end
    end
    
    // =========================================================================
    // Test 4: Mode 1 - All Different Priorities
    // =========================================================================
    $display("\n=== Test 4: Mode 1 Different Priorities ===");
    channel_priority[0] = 7;
    channel_priority[1] = 3;
    channel_priority[2] = 9;  // Highest
    channel_priority[3] = 1;
    channel_priority[4] = 5;
    channel_priority[5] = 11; // Actually highest
    channel_priority[6] = 2;
    channel_priority[7] = 4;
    
    repeat(20) @(posedge clk);
    $display("Selected: Ch%0d (should be Ch5 with priority 11)", selected_channel);
    
    // =========================================================================
    // Test 5: Mode 2 - Weighted with Accumulator Coverage
    // =========================================================================
    $display("\n=== Test 5: Mode 2 Weighted Accumulation ===");
    arbiter_mode = 2'b10;
    channel_ready = 16'h000F;  // Ch 0-3
    
    // Set different weights
    channel_weight[0] = 10;
    channel_weight[1] = 25;
    channel_weight[2] = 50;
    channel_weight[3] = 100;
    
    for (int cycle = 0; cycle < 500; cycle++) begin
        @(posedge clk);
        
        // Dynamic weight changes to trigger different comparisons
        if (cycle == 100) begin
            channel_weight[0] = 200;
            $display("  Cycle %0d: Ch0 weight -> 200", cycle);
        end
        if (cycle == 200) begin
            channel_weight[1] = 150;
            $display("  Cycle %0d: Ch1 weight -> 150", cycle);
        end
        if (cycle == 300) begin
            channel_weight[2] = 175;
            $display("  Cycle %0d: Ch2 weight -> 175", cycle);
        end
        
        if (cycle % 50 == 0) begin
            $display("  Cycle %0d: Ch%0d selected", cycle, selected_channel);
        end
    end
    
    // =========================================================================
    // Test 6: Mode 2 - Zero and Max Weights
    // =========================================================================
    $display("\n=== Test 6: Mode 2 Edge Case Weights ===");
    channel_weight[0] = 0;    // Zero
    channel_weight[1] = 1;    // Min
    channel_weight[2] = 255;  // Max
    channel_weight[3] = 128;  // Mid
    
    repeat(300) @(posedge clk);
    $display("With zero weight: Ch%0d selected", selected_channel);
    
    // =========================================================================
    // Test 7: Mode 3 - Urgent Channels
    // =========================================================================
    $display("\n=== Test 7: Mode 3 Urgent ===");
    arbiter_mode = 2'b11;
    channel_urgent = 16'h0004;  // Ch2 urgent
    channel_ready = 16'h000F;
    
    repeat(10) @(posedge clk);
    $display("With Ch2 urgent: Selected Ch%0d", selected_channel);
    
    // Multiple urgent
    channel_urgent = 16'h000E;  // Ch 1,2,3 urgent
    repeat(10) @(posedge clk);
    $display("With Ch1,2,3 urgent: Selected Ch%0d (first urgent)", selected_channel);
    
    // Clear urgent - fallback to weighted
    channel_urgent = 16'h0000;
    repeat(50) @(posedge clk);
    $display("No urgent (fallback): Selected Ch%0d", selected_channel);
    
    // =========================================================================
    // Test 8: Enable/Ready Mask Combinations
    // =========================================================================
    $display("\n=== Test 8: Enable/Ready Combinations ===");
    arbiter_mode = 2'b01;
    channel_enable = 16'hFFFF;
    
    // Various ready patterns
    channel_ready = 16'h0001;
    repeat(5) @(posedge clk);
    $display("Ready 0001: Ch%0d", selected_channel);
    
    channel_ready = 16'h0003;
    repeat(5) @(posedge clk);
    $display("Ready 0003: Ch%0d", selected_channel);
    
    channel_ready = 16'h000F;
    repeat(5) @(posedge clk);
    $display("Ready 000F: Ch%0d", selected_channel);
    
    channel_ready = 16'h00FF;
    repeat(5) @(posedge clk);
    $display("Ready 00FF: Ch%0d", selected_channel);
    
    channel_ready = 16'hFFFF;
    repeat(5) @(posedge clk);
    $display("Ready FFFF: Ch%0d", selected_channel);
    
    channel_ready = 16'h5555;  // Alternating
    repeat(5) @(posedge clk);
    $display("Ready 5555: Ch%0d", selected_channel);
    
    channel_ready = 16'hAAAA;  // Opposite alternating
    repeat(5) @(posedge clk);
    $display("Ready AAAA: Ch%0d", selected_channel);
    
    // =========================================================================
    // Test 9: found_valid_channel Flag Coverage
    // =========================================================================
    $display("\n=== Test 9: Valid Channel Flag Coverage ===");
    
    // No channels enabled
    channel_enable = 16'h0000;
    channel_ready = 16'hFFFF;
    repeat(10) @(posedge clk);
    $display("Enable 0000, Ready FFFF: valid=%0b", channel_valid);
    
    // Gradually enable channels
    channel_enable = 16'h0001;
    repeat(5) @(posedge clk);
    $display("Enable 0001: Ch%0d valid=%0b", selected_channel, channel_valid);
    
    channel_enable = 16'h0003;
    repeat(5) @(posedge clk);
    $display("Enable 0003: Ch%0d valid=%0b", selected_channel, channel_valid);
    
    channel_enable = 16'h000F;
    repeat(5) @(posedge clk);
    $display("Enable 000F: Ch%0d valid=%0b", selected_channel, channel_valid);
    
    // Enable all, but no ready
    channel_enable = 16'hFFFF;
    channel_ready = 16'h0000;
    repeat(10) @(posedge clk);
    $display("Enable FFFF, Ready 0000: valid=%0b", channel_valid);
    
    // =========================================================================
    // Test 10: ADC Busy Impact
    // =========================================================================
    $display("\n=== Test 10: ADC Busy ===");
    channel_enable = 16'hFFFF;
    channel_ready = 16'h000F;
    arbiter_mode = 2'b00;
    
    adc_busy = 0;
    repeat(5) @(posedge clk);
    $display("ADC not busy: valid=%0b Ch%0d", channel_valid, selected_channel);
    
    adc_busy = 1;
    repeat(5) @(posedge clk);
    $display("ADC busy: valid=%0b Ch%0d", channel_valid, selected_channel);
    
    adc_busy = 0;
    repeat(5) @(posedge clk);
    $display("ADC not busy again: valid=%0b Ch%0d", channel_valid, selected_channel);
    
    // =========================================================================
    // Test 11: Channel Accept Handshake
    // =========================================================================
    $display("\n=== Test 11: Channel Accept Handshake ===");
    arbiter_mode = 2'b00;
    channel_ready = 16'h000F;
    
    for (int i = 0; i < 10; i++) begin
        @(posedge clk);
        if (channel_valid) begin
            $display("  Ch%0d selected, accepting...", selected_channel);
            channel_accept = 1;
            @(posedge clk);
            channel_accept = 0;
            @(posedge clk);
        end
    end
    
    $display("\n=== ALL TESTS COMPLETE ===");
    
    $stop;
end

endmodule

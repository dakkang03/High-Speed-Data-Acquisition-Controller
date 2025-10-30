`timescale 1ns/1ps

module tb_fifo;

logic clk, rst_n;
logic [1:0] fifo_mode;
logic [7:0] watermark_l1, watermark_l2, watermark_l3;
logic [15:0] wr_data;
logic wr_en, wr_full;
logic [1:0] wr_level;
logic [15:0] rd_data;
logic rd_en, rd_empty;
logic [1:0] rd_level;
logic [31:0] fifo_status;
logic [2:0] level_overflow;
logic backpressure_active;
logic [9:0] count;

single_fifo #(
    .DATA_WIDTH(16),
    .FIFO_DEPTH(672)
) dut (.*);

initial clk = 0;
always #5 clk = ~clk;

initial begin
    rst_n = 0;
    fifo_mode = 2'b00;
    watermark_l1 = 8'd75;
    watermark_l2 = 8'd80;
    watermark_l3 = 8'd90;
    wr_data = 0;
    wr_en = 0;
    rd_en = 0;
    
    repeat(10) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);
    
    // Test 1: Fill FIFO to check level transitions
    $display("=== Test 1: Level Transitions ===");
    for (int i = 0; i < 100; i++) begin
        @(posedge clk);
        wr_en = 1;
        wr_data = i;
        
        if (count == 32) $display("L1 threshold: count=%0d", count);
        if (count == 160) $display("L2 threshold: count=%0d", count);
        if (count == 400) $display("L3 threshold: count=%0d", count);
    end
    wr_en = 0;
    
    // Test 2: Read until empty
    $display("\n=== Test 2: Drain to Empty ===");
    repeat(10) @(posedge clk);
    for (int i = 0; i < 150; i++) begin
        @(posedge clk);
        rd_en = 1;
        if (rd_empty) begin
            $display("Empty at cycle %0d", i);
            break;
        end
    end
    rd_en = 0;
    
    // Test 3: Fill to Full
    $display("\n=== Test 3: Fill to Full ===");
    repeat(10) @(posedge clk);
    for (int i = 0; i < 700; i++) begin
        @(posedge clk);
        wr_en = 1;
        wr_data = i;
        if (wr_full) begin
            $display("Full at count=%0d", count);
            break;
        end
    end
    wr_en = 0;
    
    // Test 4: Backpressure (90%)
    $display("\n=== Test 4: Backpressure ===");
    rst_n = 0;
    repeat(5) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);
    
    for (int i = 0; i < 650; i++) begin
        @(posedge clk);
        wr_en = 1;
        wr_data = i;
        if (backpressure_active) begin
            $display("Backpressure at count=%0d", count);
            break;
        end
    end
    wr_en = 0;
    
    // Test 5: Overflow flags
    $display("\n=== Test 5: Overflow Flags ===");
    repeat(10) @(posedge clk);
    for (int i = 0; i < 700; i++) begin
        @(posedge clk);
        wr_en = 1;
        if (level_overflow[0]) $display("L1 overflow: %03b", level_overflow);
        if (level_overflow[1]) $display("L2 overflow: %03b", level_overflow);
        if (level_overflow[2]) $display("L3 overflow: %03b", level_overflow);
    end
    wr_en = 0;
    
    // Test 6: Simultaneous read/write
    $display("\n=== Test 6: Simultaneous R/W ===");
    rst_n = 0;
    repeat(5) @(posedge clk);
    rst_n = 1;
    
    // Fill halfway
    for (int i = 0; i < 300; i++) begin
        @(posedge clk);
        wr_en = 1;
        wr_data = i;
    end
    
    // Simultaneous
    for (int i = 0; i < 100; i++) begin
        @(posedge clk);
        wr_en = 1;
        rd_en = 1;
        wr_data = 300 + i;
    end
    wr_en = 0;
    rd_en = 0;
    
    $display("\n=== FIFO Tests Complete ===");
    $finish;
end

endmodule

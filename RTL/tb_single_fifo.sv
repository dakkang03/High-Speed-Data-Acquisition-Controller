module tb_single_fifo;

localparam DATA_WIDTH = 16;
localparam L1_DEPTH = 32;
localparam L2_DEPTH = 128;
localparam L3_DEPTH = 512;
localparam CLK_PERIOD = 10;

logic clk, rst_n;
logic [1:0] fifo_mode;
logic [7:0] watermark_l1, watermark_l2, watermark_l3;
logic [DATA_WIDTH-1:0] wr_data, rd_data;
logic wr_en, rd_en, wr_full, rd_empty;
logic [1:0] wr_level, rd_level;
logic [31:0] fifo_status;
logic [2:0] level_overflow;
logic backpressure_active;

int errors;

always #(CLK_PERIOD/2) clk = ~clk;

hierarchical_fifo_system #(
    .DATA_WIDTH(DATA_WIDTH),
    .L1_DEPTH(L1_DEPTH),
    .L2_DEPTH(L2_DEPTH),
    .L3_DEPTH(L3_DEPTH)
) dut (.*);

task automatic write_data(input int count, input logic [15:0] base_val);
    int i;
    for (i = 0; i < count; i = i + 1) begin
        @(posedge clk);
        if (!wr_full) begin
            wr_data = base_val + i;
            wr_en = 1;
        end else begin
            $display("ERROR: FIFO full at write %0d", i);
            errors = errors + 1;
            i = count;
        end
    end
    @(posedge clk);
    wr_en = 0;
endtask

task automatic read_and_check(input int count, input logic [15:0] expected_base);
    int i;
    logic [15:0] expected, actual;
    for (i = 0; i < count; i = i + 1) begin
        expected = expected_base + i;
        @(posedge clk);
        if (!rd_empty) begin
            rd_en = 1;
            #1;
            actual = rd_data;
            if (actual !== expected) begin
                $display("ERROR: Read[%0d] Expected 0x%04h, got 0x%04h", i, expected, actual);
                errors = errors + 1;
            end
        end else begin
            $display("ERROR: FIFO empty at read %0d", i);
            errors = errors + 1;
            i = count;
        end
    end
    @(posedge clk);
    rd_en = 0;
endtask

task automatic drain_all();
    int count;
    count = 0;
    $display("  [Draining all data...]");
    while (!rd_empty) begin
        @(posedge clk);
        rd_en = 1;
        count = count + 1;
    end
    @(posedge clk);
    rd_en = 0;
    $display("  [Drained %0d entries]", count);
    #(CLK_PERIOD*5);
endtask

task automatic check_empty();
    if (!rd_empty) begin
        $display("ERROR: FIFO not empty after drain!");
        errors = errors + 1;
    end
    if (dut.l1_count != 0 || dut.l2_count != 0 || dut.l3_count != 0) begin
        $display("ERROR: FIFO counts not zero: L1=%0d L2=%0d L3=%0d", 
                 dut.l1_count, dut.l2_count, dut.l3_count);
        errors = errors + 1;
    end
endtask

initial begin
    clk = 0; 
    rst_n = 0;
    fifo_mode = 2'b10;
    watermark_l1 = 75; 
    watermark_l2 = 80; 
    watermark_l3 = 90;
    wr_data = 0; 
    wr_en = 0; 
    rd_en = 0;
    errors = 0;
    
    $display("=== Hierarchical FIFO Test Started ===\n");
    
    #(CLK_PERIOD*5) rst_n = 1;
    #(CLK_PERIOD*5);
    
    //=========================================================================
    // Test 1: Basic Write/Read
    //=========================================================================
    $display("--- Test 1: Basic Write/Read (20 entries) ---");
    
    write_data(20, 16'h0000);
    #(CLK_PERIOD*2);
    $display("After Write: L1=%0d, L2=%0d, L3=%0d", 
             dut.l1_count, dut.l2_count, dut.l3_count);
    
    read_and_check(20, 16'h0000);
    #(CLK_PERIOD*2);
    
    check_empty();
    $display("Test 1 Complete\n");
    
    //=========================================================================
    // Test 2: L1 Overflow to L2
    //=========================================================================
    $display("--- Test 2: Fill L1 (50 entries) -> Promotion to L2 ---");
    
    write_data(50, 16'h0100);
    #(CLK_PERIOD*10);
    $display("After Write: L1=%0d, L2=%0d, L3=%0d", 
             dut.l1_count, dut.l2_count, dut.l3_count);
    
    if (dut.l2_count > 0) begin
        $display("Promotion to L2 successful");
    end else begin
        $display("ERROR: No promotion to L2");
        errors = errors + 1;
    end
    
    read_and_check(50, 16'h0100);
    drain_all();
    check_empty();
    $display("Test 2 Complete\n");
    
    //=========================================================================
    // Test 3: Large Burst - All 3 Levels
    //=========================================================================
    $display("--- Test 3: Large Burst (200 entries) -> Use All 3 Levels ---");
    
    write_data(200, 16'h0200);
    #(CLK_PERIOD*20);
    $display("After Write: L1=%0d, L2=%0d, L3=%0d", 
             dut.l1_count, dut.l2_count, dut.l3_count);
    
    if (dut.l3_count > 0) begin
        $display("All 3 levels in use");
    end else begin
        $display("ERROR: L3 not used");
        errors = errors + 1;
    end
    
    $display("Verifying first 20 reads...");
    read_and_check(20, 16'h0200);
    
    drain_all();
    check_empty();
    $display("Test 3 Complete\n");
    
    //=========================================================================
    // Test 4: Maximum Capacity
    //=========================================================================
    $display("--- Test 4: Maximum Capacity Test ---");
    
    write_data(672, 16'h0400);
    #(CLK_PERIOD*20);
    
    $display("Total Capacity: L1=%0d + L2=%0d + L3=%0d = %0d", 
             dut.l1_count, dut.l2_count, dut.l3_count,
             dut.l1_count + dut.l2_count + dut.l3_count);
    
    if ((dut.l1_count + dut.l2_count + dut.l3_count) >= 600) begin
        $display("Capacity test passed");
    end else begin
        $display("ERROR: Capacity too low");
        errors = errors + 1;
    end
    
    drain_all();
    check_empty();
    $display("Test 4 Complete\n");
    
    //=========================================================================
    // Summary
    //=========================================================================
    #(CLK_PERIOD*10);
    
    $display("\n=== Test Summary ===");
    $display("Total Errors: %0d", errors);
    if (errors == 0) begin
        $display("*** ALL TESTS PASSED ***");
    end else begin
        $display("*** TESTS FAILED ***");
    end
    
    $finish;
end

always_ff @(posedge clk) begin
    if (rst_n && wr_en && wr_full) begin
        $display("WARNING: Write attempt when full at %0t", $time);
    end
end

initial begin
    $dumpfile("tb_hierarchical_fifo_system.vcd");
    $dumpvars(0, tb_hierarchical_fifo_system);
end

endmodule

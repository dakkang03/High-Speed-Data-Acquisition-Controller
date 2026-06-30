// =============================================================================
// tb_fifo.sv
// single_fifo
//  - Directed test: overflow, underflow, simultaneous read/write
//  - SVA: count/full/empty invariant, overflow/underflow, data integrity
//  - Functional coverage: FIFO_DEPTH=16, almost_full threshold=12
// =============================================================================
`timescale 1ns/1ps

module tb_single_fifo;

    localparam DATA_WIDTH = 16;
    localparam FIFO_DEPTH = 16;
    localparam ALMOST_FULL_THRESHOLD = 12;
    localparam ADDR_WIDTH = $clog2(FIFO_DEPTH);

    logic clk = 0;
    logic rst_n;

    logic [DATA_WIDTH-1:0] wr_data;
    logic wr_en;
    logic wr_full;
    logic almost_full;

    logic [DATA_WIDTH-1:0] rd_data;
    logic rd_en;
    logic rd_empty;

    logic [ADDR_WIDTH:0] count;

    always #5 clk = ~clk; // 100MHz

    single_fifo #(
        .DATA_WIDTH(DATA_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .ALMOST_FULL_THRESHOLD(ALMOST_FULL_THRESHOLD)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .wr_data(wr_data), .wr_en(wr_en), .wr_full(wr_full), .almost_full(almost_full),
        .rd_data(rd_data), .rd_en(rd_en), .rd_empty(rd_empty),
        .count(count)
    );

    // =========================================================================
    // Reference model (golden queue)
    // =========================================================================
    logic [DATA_WIDTH-1:0] ref_q [$];
    int sb_checks = 0;
    int sb_errors = 0;

    // 통계
    int stat_overflow_attempts   = 0; // wr_en && wr_full
    int stat_underflow_attempts  = 0; // rd_en && rd_empty
    int stat_simultaneous_rw     = 0; // wr_en && !wr_full && rd_en && !rd_empty

    // =========================================================================
    // SVA
    // =========================================================================
    // A1: count in 0..FIFO_DEPTH range
    property p_count_range;
        @(posedge clk) disable iff (!rst_n)
        (count >= 0) && (count <= FIFO_DEPTH);
    endproperty
    assert property (p_count_range)
        else $error("[SVA-A1][%0t] count out of range: count=%0d", $time, count);

    // A2: wr_full <-> count == FIFO_DEPTH
    property p_wr_full_consistency;
        @(posedge clk) disable iff (!rst_n)
        wr_full == (count == FIFO_DEPTH);
    endproperty
    assert property (p_wr_full_consistency)
        else $error("[SVA-A2][%0t] wr_full inconsistent: wr_full=%0b count=%0d", $time, wr_full, count);

    // A3: rd_empty <-> count == 0
    property p_rd_empty_consistency;
        @(posedge clk) disable iff (!rst_n)
        rd_empty == (count == 0);
    endproperty
    assert property (p_rd_empty_consistency)
        else $error("[SVA-A3][%0t] rd_empty inconsistent: rd_empty=%0b count=%0d", $time, rd_empty, count);

    // A4: almost_full <-> count >= ALMOST_FULL_THRESHOLD
    property p_almost_full_consistency;
        @(posedge clk) disable iff (!rst_n)
        almost_full == (count >= ALMOST_FULL_THRESHOLD);
    endproperty
    assert property (p_almost_full_consistency)
        else $error("[SVA-A4][%0t] almost_full inconsistent: almost_full=%0b count=%0d", $time, almost_full, count);

    // A5: wr_full, the count does not increase even if wr_en is present (prevents overflow)
    property p_no_overflow;
        @(posedge clk) disable iff (!rst_n)
        (wr_en && wr_full && !(rd_en && !rd_empty)) |=> (count == $past(count));
    endproperty
    assert property (p_no_overflow)
        else $error("[SVA-A5][%0t] overflow: count changed while full and no read", $time);

    // A6: when rd_empty, the count does not decrease even if rd_en exists (prevents underflow)
    property p_no_underflow;
        @(posedge clk) disable iff (!rst_n)
        (rd_en && rd_empty && !(wr_en && !wr_full)) |=> (count == $past(count));
    endproperty
    assert property (p_no_underflow)
        else $error("[SVA-A6][%0t] underflow: count changed while empty and no write", $time);

    // A7: The count can change by a maximum of ±1 per clock cycle (simultaneous R/W=0, single R or W=±1, simultaneous success R/W=0)
    property p_count_delta;
        @(posedge clk) disable iff (!rst_n)
        ($past(rst_n)) |-> ((count - $past(count)) inside {-1, 0, 1});
    endproperty
    assert property (p_count_delta)
        else $error("[SVA-A7][%0t] count changed by more than 1: count=%0d past=%0d", $time, count, $past(count));

    // A8: reset, count==0, rd_empty==1, wr_full==0
    property p_reset_state;
        @(posedge clk) (!rst_n) |=> (count == 0 && rd_empty == 1'b1 && wr_full == 1'b0);
    endproperty
    assert property (p_reset_state)
        else $error("[SVA-A8][%0t] reset state incorrect: count=%0d rd_empty=%0b wr_full=%0b", $time, count, rd_empty, wr_full);

    // =========================================================================
    // Functional coverage (manual covergroup)
    // =========================================================================
    covergroup cg_fifo @(posedge clk);
        option.per_instance = 1;
        cp_count: coverpoint count {
            bins empty       = {0};
            bins low         = {[1:6]};
            bins mid         = {[7:11]};
            bins almost_full = {[12:15]};
            bins full        = {16};
        }
        cp_wr_en   : coverpoint wr_en;
        cp_rd_en   : coverpoint rd_en;
        cp_simul_rw: cross cp_wr_en, cp_rd_en {
            bins both_active = binsof(cp_wr_en) intersect {1} && binsof(cp_rd_en) intersect {1};
        }
        cp_overflow_attempt : coverpoint (wr_en && wr_full) {
            bins occurred = {1};
        }
        cp_underflow_attempt: coverpoint (rd_en && rd_empty) {
            bins occurred = {1};
        }
    endgroup

    cg_fifo cg = new();

    // =========================================================================
    // Reference model update (scoreboard)
    // =========================================================================
    logic do_write, do_read;
    always @(posedge clk) begin
        #1;
        if (!rst_n) begin
            ref_q.delete();
        end else begin
        // -----------------------------------------------------------------
        // 0. Perform count comparison before updating ref_q.
        // The count for the current clock is a value that "reflects write/read operations up to the previous clock,"
        // and since ref_q has not yet reflected the do_write/do_read operations for this clock,
        // at this point, the two must be equal. (Compare first -> then update ref_q)
        // -----------------------------------------------------------------
            if (count !== ref_q.size()) begin
                sb_errors++;
                $error("[SB-FIFO][%0t] count mismatch: dut=%0d ref=%0d", $time, count, ref_q.size());
            end

            do_write = wr_en && !wr_full;
            do_read  = rd_en && !rd_empty;

            if (wr_en && wr_full)  stat_overflow_attempts++;
            if (rd_en && rd_empty) stat_underflow_attempts++;
            if (do_write && do_read) stat_simultaneous_rw++;

            if (do_read) begin
                sb_checks++;
                if (ref_q.size() == 0 || rd_data !== ref_q[0]) begin
                    sb_errors++;
                    $error("[SB-FIFO][%0t] read mismatch: got=%h expected=%h",
                           $time, rd_data, (ref_q.size() > 0) ? ref_q[0] : 'x);
                end else begin
                    ref_q.pop_front();
                end
            end

            if (do_write) begin
                ref_q.push_back(wr_data);
            end
        end
    end

    // =========================================================================
    // Stimulus
    // =========================================================================
    initial begin
        rst_n = 0;
        wr_data = 0; wr_en = 0; rd_en = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 1: Overflow
        // -----------------------------------------------------------------
        $display("[TB-FIFO] Scenario 1: Overflow test");
        for (int i = 0; i < FIFO_DEPTH + 4; i++) begin
            wr_data = i;
            wr_en   = 1;
            rd_en   = 0;
            @(posedge clk);
        end
        wr_en = 0;
        @(posedge clk);
        if (count !== FIFO_DEPTH)
            $error("[TB-FIFO] Scenario1 FAIL: count=%0d expected=%0d", count, FIFO_DEPTH);
        else
            $display("[TB-FIFO] Scenario1 PASS: count=%0d (FIFO correctly saturated)", count);

        // -----------------------------------------------------------------
        // Scenario 2: Underflow
        // -----------------------------------------------------------------
        $display("[TB-FIFO] Scenario 2: Underflow test");
        // 먼저 전부 비우기
        rd_en = 1;
        repeat (FIFO_DEPTH + 4) @(posedge clk); // FIFO_DEPTH번 후엔 underflow 시도
        rd_en = 0;
        @(posedge clk);
        if (count !== 0)
            $error("[TB-FIFO] Scenario2 FAIL: count=%0d expected=0", count);
        else
            $display("[TB-FIFO] Scenario2 PASS: count=0 (FIFO correctly emptied, underflow attempts ignored)");

        // -----------------------------------------------------------------
        // Scenario 3: Simultaneous read/write (steady-state streaming)
        // -----------------------------------------------------------------
        $display("[TB-FIFO] Scenario 3: Simultaneous read/write test");
        // 먼저 절반 채움
        rd_en = 0;
        for (int i = 0; i < FIFO_DEPTH/2; i++) begin
            wr_data = 100 + i;
            wr_en   = 1;
            @(posedge clk);
        end
        for (int i = 0; i < 20; i++) begin
            wr_data = 200 + i;
            wr_en   = 1;
            rd_en   = 1;
            @(posedge clk);
        end
        wr_en = 0; rd_en = 0;
        @(posedge clk);
        $display("[TB-FIFO] Scenario3 PASS: simultaneous R/W ran for 20 cycles, count=%0d", count);

        // -----------------------------------------------------------------
        // Scenario 4: wr_en/rd_en asserted during reset
        // -----------------------------------------------------------------
        $display("[TB-FIFO] Scenario 4: Inputs asserted during reset");
        wr_data = 16'hDEAD;
        wr_en   = 1;
        rd_en   = 1;
        rst_n   = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        if (count !== 0 || rd_empty !== 1'b1 || wr_full !== 1'b0)
            $error("[TB-FIFO] Scenario4 FAIL: count=%0d rd_empty=%0b wr_full=%0b after reset",
                   count, rd_empty, wr_full);
        else
            $display("[TB-FIFO] Scenario4 PASS: FIFO correctly reset despite asserted inputs");
        wr_en = 0; rd_en = 0;
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 5: Randomized read/write (constrained-random)
        // -----------------------------------------------------------------
        $display("[TB-FIFO] Scenario 5: Constrained-random read/write");
        for (int i = 0; i < 500; i++) begin
            wr_data = $urandom_range(0, (1 << DATA_WIDTH) - 1);
            wr_en   = $urandom_range(0, 1);
            rd_en   = $urandom_range(0, 1);
            @(posedge clk);
        end
        wr_en = 0; rd_en = 0;
        
        rd_en = 1;
        repeat (FIFO_DEPTH + 2) @(posedge clk);
        rd_en = 0;
        @(posedge clk);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("==============================================");
        $display("FIFO scoreboard: checks=%0d errors=%0d", sb_checks, sb_errors);
        $display("Overflow attempts observed : %0d", stat_overflow_attempts);
        $display("Underflow attempts observed: %0d", stat_underflow_attempts);
        $display("Simultaneous R/W cycles     : %0d", stat_simultaneous_rw);
        if (sb_errors == 0 && stat_overflow_attempts > 0 && stat_underflow_attempts > 0
            && stat_simultaneous_rw > 0)
            $display("FIFO TEST PASSED");
        else
            $display("FIFO TEST FAILED");
        $display("==============================================");

        $finish;
    end

endmodule

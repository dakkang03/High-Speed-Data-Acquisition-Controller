// =============================================================================
// tb_high_speed_daq_controller.sv  (ModelSim-Altera Starter Edition ?? ??)
//   - covergroup, assert property, randomize/randcase/randsequence ???
//   - ??? ?? ??? "?? ??? + always ?? ? if/$display"? ??
//   - $urandom / $urandom_range ? ?? (Starter?? ???? ?? system function)
// =============================================================================

`timescale 1ns/1ps

module tb_high_speed_daq_controller;

  localparam NUM_CHANNELS  = 8;
  localparam CHANNEL_WIDTH = $clog2(NUM_CHANNELS);
  localparam FIFO_DEPTH    = 16;

  // ---------------------------------------------------------------------
  // Clock / Reset
  // ---------------------------------------------------------------------
  logic clk;
  logic rst_n;

  initial clk = 0;
  always #5 clk = ~clk; // 100MHz

  // ---------------------------------------------------------------------
  // DUT 1: configurable_arbiter
  // ---------------------------------------------------------------------
  logic [1:0] arbiter_mode;
  logic [3:0] channel_priority [NUM_CHANNELS-1:0];
  logic [7:0] channel_weight   [NUM_CHANNELS-1:0];
  logic [NUM_CHANNELS-1:0] channel_enable;
  logic [NUM_CHANNELS-1:0] channel_ready;
  logic [NUM_CHANNELS-1:0] channel_urgent;
  logic adc_busy;

  logic [CHANNEL_WIDTH-1:0] selected_channel;
  logic channel_valid;
  logic channel_accept;

  configurable_arbiter #(.NUM_CHANNELS(NUM_CHANNELS)) dut_arb (
    .clk(clk), .rst_n(rst_n),
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

  // ---------------------------------------------------------------------
  // DUT 2: single_fifo
  // ---------------------------------------------------------------------
  logic [15:0] wr_data, rd_data;
  logic wr_en, rd_en;
  logic wr_full, rd_empty, almost_full;
  logic [$clog2(FIFO_DEPTH):0] count;

  single_fifo #(
    .DATA_WIDTH(16), .FIFO_DEPTH(FIFO_DEPTH), .ALMOST_FULL_THRESHOLD(12)
  ) dut_fifo (
    .clk(clk), .rst_n(rst_n),
    .wr_data(wr_data), .wr_en(wr_en), .wr_full(wr_full), .almost_full(almost_full),
    .rd_data(rd_data), .rd_en(rd_en), .rd_empty(rd_empty),
    .count(count)
  );

  // =========================================================================
  // ARBITER ??: Reference Model + Scoreboard
  // =========================================================================
  logic [7:0] ref_weight_acc [NUM_CHANNELS-1:0];
  logic [CHANNEL_WIDTH-1:0] ref_rr_counter;

  function automatic [CHANNEL_WIDTH-1:0] ref_find_next_rr(
      input [CHANNEL_WIDTH-1:0] start,
      input [NUM_CHANNELS-1:0] en, rdy
  );
    for (int i = 0; i < NUM_CHANNELS; i++) begin
      int idx = (start + i) % NUM_CHANNELS;
      if (en[idx] && rdy[idx]) return idx;
    end
    return start;
  endfunction

  function automatic [CHANNEL_WIDTH-1:0] ref_find_max_weight(
      input logic [7:0] w [NUM_CHANNELS-1:0],
      input [NUM_CHANNELS-1:0] en, rdy
  );
    logic [7:0] max_w = 0;
    logic [CHANNEL_WIDTH-1:0] max_ch = 0;
    for (int i = 0; i < NUM_CHANNELS; i++)
      if (en[i] && rdy[i] && w[i] > max_w) begin max_w = w[i]; max_ch = i; end
    return max_ch;
  endfunction

  function automatic [CHANNEL_WIDTH-1:0] ref_find_priority(
      input logic [3:0] p [NUM_CHANNELS-1:0],
      input [NUM_CHANNELS-1:0] en, rdy
  );
    logic [3:0] max_p = 0;
    logic [CHANNEL_WIDTH-1:0] max_ch = 0;
    logic found = 0;
    for (int i = 0; i < NUM_CHANNELS; i++)
      if (en[i] && rdy[i]) begin
        if (!found || p[i] > max_p) begin max_p = p[i]; max_ch = i; found = 1; end
      end
    return max_ch;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < NUM_CHANNELS; i++) ref_weight_acc[i] <= 0;
      ref_rr_counter <= 0;
    end else begin
      for (int i = 0; i < NUM_CHANNELS; i++)
        if (channel_enable[i]) ref_weight_acc[i] <= ref_weight_acc[i] + channel_weight[i];

      if (channel_accept && arbiter_mode == 2'b10)
        ref_weight_acc[selected_channel] <= 0;

      if (channel_accept && arbiter_mode == 2'b00)
        ref_rr_counter <= (ref_rr_counter == NUM_CHANNELS-1) ? 0 : ref_rr_counter + 1;
    end
  end

  logic [CHANNEL_WIDTH-1:0] ref_selected;
  logic ref_valid;
  int sb_arb_checks, sb_arb_errors;

  always_comb begin
    case (arbiter_mode)
      2'b00: ref_selected = ref_find_next_rr(ref_rr_counter, channel_enable, channel_ready);
      2'b01: ref_selected = ref_find_priority(channel_priority, channel_enable, channel_ready);
      2'b10: ref_selected = ref_find_max_weight(ref_weight_acc, channel_enable, channel_ready);
      default: ref_selected = ref_find_max_weight(ref_weight_acc, channel_enable, channel_ready);
    endcase
    ref_valid = channel_enable[ref_selected] && channel_ready[ref_selected] && !adc_busy;
  end

  always @(posedge clk) begin
    if (rst_n) begin
      sb_arb_checks = sb_arb_checks + 1;
      if (channel_valid) begin
        if (selected_channel !== ref_selected || channel_valid !== ref_valid) begin
          sb_arb_errors = sb_arb_errors + 1;
          $display("[SB-ARB][%0t] MISMATCH: mode=%0d DUT(sel=%0d,valid=%0b) REF(sel=%0d,valid=%0b)",
                   $time, arbiter_mode, selected_channel, channel_valid, ref_selected, ref_valid);
        end
      end
    end
  end

  // =========================================================================
  // FIFO ??: Reference Model (??+???, queue ??) + Scoreboard
  // =========================================================================
  // Starter Edition?? ?? queue([$]) ??? -> ?? ?? + head/tail? ??
  logic [15:0] ref_fifo_mem [0:255]; // ??? ? ?? ??
  int ref_head, ref_tail, ref_size;
  int sb_fifo_checks, sb_fifo_errors;

  task ref_fifo_reset();
    ref_head = 0; ref_tail = 0; ref_size = 0;
  endtask

  task ref_fifo_push(input logic [15:0] d);
    ref_fifo_mem[ref_tail] = d;
    ref_tail = (ref_tail + 1) % 256;
    ref_size = ref_size + 1;
  endtask

  function automatic logic [15:0] ref_fifo_pop();
    logic [15:0] v;
    v = ref_fifo_mem[ref_head];
    ref_head = (ref_head + 1) % 256;
    ref_size = ref_size - 1;
    return v;
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      ref_fifo_reset();
    end else begin
      // ---- DUT? count/wr_full/rd_empty? nonblocking ???? ?? edge ?? ???
      //      "?? ?"? ??? ?? ?? -> push/pop ??? ref_size? ???? ?? ??
      //      ?, ????? ?? ??(? reset ?? ?) DUT ????? X? ? ???? skip ----
      if (!$isunknown(rd_empty) && (ref_size == 0) !== rd_empty) begin
        sb_fifo_errors = sb_fifo_errors + 1;
        $display("[SB-FIFO][%0t] EMPTY mismatch: ref_size=%0d rd_empty=%0b", $time, ref_size, rd_empty);
      end
      if (!$isunknown(wr_full) && (ref_size >= FIFO_DEPTH) !== wr_full) begin
        sb_fifo_errors = sb_fifo_errors + 1;
        $display("[SB-FIFO][%0t] FULL mismatch: ref_size=%0d wr_full=%0b", $time, ref_size, wr_full);
      end

      if (wr_en && !wr_full) ref_fifo_push(wr_data);

      if (rd_en && !rd_empty) begin
        sb_fifo_checks = sb_fifo_checks + 1;
        if (ref_size > 0) begin
          logic [15:0] expected;
          expected = ref_fifo_pop();
          if (expected !== rd_data) begin
            sb_fifo_errors = sb_fifo_errors + 1;
            $display("[SB-FIFO][%0t] MISMATCH: expected=%h got=%h", $time, expected, rd_data);
          end
        end
      end
    end
  end

  // =========================================================================
  // "Assertions" ??: always ?? + if/$display (A1~A10)
  // =========================================================================
  int err_A1, err_A2, err_A3, err_A4, err_A5, err_A6, err_A7, err_A8, err_A9, err_A10;

  // ?? ??? ? ???
  logic [$clog2(FIFO_DEPTH):0] count_prev;
  logic wr_full_prev, rd_empty_prev, wr_en_prev, rd_en_prev;
  logic channel_accept_prev;
  logic [1:0] arbiter_mode_prev;
  logic [CHANNEL_WIDTH-1:0] selected_channel_prev;
  logic [CHANNEL_WIDTH-1:0] rr_counter_prev;
  logic rst_n_prev;

  always @(posedge clk) begin
    // ---- A1: full && empty ?? ?? ----
    if (rst_n) begin
      if (wr_full && rd_empty) begin
        err_A1 = err_A1 + 1;
        $display("[A1][%0t] FIFO full and empty simultaneously", $time);
      end

      // ---- A2: count <= FIFO_DEPTH ----
      if (count > FIFO_DEPTH) begin
        err_A2 = err_A2 + 1;
        $display("[A2][%0t] FIFO count exceeds depth: %0d", $time, count);
      end

      // ---- A3: ?? ??? (full && wr_en) ???, ?? ?? count==FIFO_DEPTH (overflow ??) ----
      if (rst_n_prev && wr_full_prev && wr_en_prev) begin
        if (count != FIFO_DEPTH) begin
          err_A3 = err_A3 + 1;
          $display("[A3][%0t] FIFO overflow: count=%0d (expected %0d)", $time, count, FIFO_DEPTH);
        end
      end

      // ---- A4: ?? ??? (empty && rd_en) ??? read? ????? ?.
      //      ?, ??? valid write? ???? count? +1 ?? ? ?? ----
      if (rst_n_prev && rd_empty_prev && rd_en_prev) begin
        logic write_happened;
        write_happened = wr_en_prev && !wr_full_prev;
        if (write_happened) begin
          if (count != count_prev + 1) begin
            err_A4 = err_A4 + 1;
            $display("[A4][%0t] FIFO underflow(with concurrent write): count %0d -> %0d (expected %0d)",
                     $time, count_prev, count, count_prev+1);
          end
        end else begin
          if (count != count_prev) begin
            err_A4 = err_A4 + 1;
            $display("[A4][%0t] FIFO underflow: count changed from %0d to %0d", $time, count_prev, count);
          end
        end
      end

      // ---- A5: simultaneous R/W (? ? ???? ??) -> count ?? ??? ? ----
      if (rst_n_prev && wr_en_prev && rd_en_prev && !wr_full_prev && !rd_empty_prev) begin
        if (count != count_prev) begin
          err_A5 = err_A5 + 1;
          $display("[A5][%0t] simultaneous R/W count delta unexpected: %0d -> %0d", $time, count_prev, count);
        end
      end

      // ---- A6: almost_full == (count>=12) ----
      if (almost_full != (count >= 12)) begin
        err_A6 = err_A6 + 1;
        $display("[A6][%0t] almost_full incorrect for count=%0d (almost_full=%0b)", $time, count, almost_full);
      end

      // ---- A7: channel_valid -> selected channel enable&&ready ----
      if (channel_valid && !(channel_enable[selected_channel] && channel_ready[selected_channel])) begin
        err_A7 = err_A7 + 1;
        $display("[A7][%0t] selected channel %0d not enable/ready", $time, selected_channel);
      end

      // ---- A8: adc_busy -> !channel_valid ----
      if (adc_busy && channel_valid) begin
        err_A8 = err_A8 + 1;
        $display("[A8][%0t] channel_valid asserted while adc_busy", $time);
      end

      // ---- A10: RR ???? accept? ?? ??? rr_counter == rr_counter_prev+1 (mod N)
      //      (DUT ??: rr_counter <= (rr_counter==N-1)?0:rr_counter+1, selected_channel?? ??) ----
      if (rst_n_prev && channel_accept_prev && arbiter_mode_prev == 2'b00) begin
        logic [CHANNEL_WIDTH-1:0] expected_rr;
        expected_rr = (rr_counter_prev == NUM_CHANNELS-1) ? 0 : rr_counter_prev + 1;
        if (dut_arb.rr_counter != expected_rr) begin
          err_A10 = err_A10 + 1;
          $display("[A10][%0t] RR counter mismatch: got=%0d expected=%0d (prev=%0d)",
                   $time, dut_arb.rr_counter, expected_rr, rr_counter_prev);
        end
      end
    end

    // ---- A9: reset ?? ? ??? count==0 ----
    if (!rst_n_prev && rst_n) begin
      if (count != 0) begin
        err_A9 = err_A9 + 1;
        $display("[A9][%0t] FIFO count not 0 right after reset: %0d", $time, count);
      end
    end

    // ?? ? ??
    count_prev            <= count;
    wr_full_prev          <= wr_full;
    rd_empty_prev         <= rd_empty;
    wr_en_prev            <= wr_en;
    rd_en_prev            <= rd_en;
    channel_accept_prev   <= channel_accept;
    arbiter_mode_prev     <= arbiter_mode;
    selected_channel_prev <= selected_channel;
    rr_counter_prev       <= dut_arb.rr_counter;
    rst_n_prev            <= rst_n;
  end

  // =========================================================================
  // Coverage ??: ?? ??? (? ???? 1? ?? ?? ??)
  // =========================================================================
  int cov_mode_rr, cov_mode_prio, cov_mode_wgt, cov_mode_dyn;
  int cov_mode_transitions [0:15]; // 4x4 = 16?? ??
  int cov_all_channels_req, cov_no_channels_req;
  int cov_reset_with_req;

  int cov_fifo_overflow_attempt, cov_fifo_underflow_attempt, cov_fifo_simul_rw;
  int cov_fifo_almost_full;
  int cov_fifo_count_empty, cov_fifo_count_low, cov_fifo_count_almost, cov_fifo_count_full;

  always @(posedge clk) begin
    // arbiter mode coverage
    case (arbiter_mode)
      2'b00: cov_mode_rr   = cov_mode_rr + 1;
      2'b01: cov_mode_prio = cov_mode_prio + 1;
      2'b10: cov_mode_wgt  = cov_mode_wgt + 1;
      2'b11: cov_mode_dyn  = cov_mode_dyn + 1;
    endcase

    // mode transition coverage
    if (rst_n_prev) begin
      int idx;
      idx = arbiter_mode_prev * 4 + arbiter_mode;
      cov_mode_transitions[idx] = cov_mode_transitions[idx] + 1;
    end

    // all/none channel request
    if ((channel_ready & channel_enable) == 8'hFF) cov_all_channels_req = cov_all_channels_req + 1;
    if ((channel_ready & channel_enable) == 8'h00) cov_no_channels_req  = cov_no_channels_req  + 1;

    // reset with input
    if (!rst_n && (|channel_ready)) cov_reset_with_req = cov_reset_with_req + 1;

    // FIFO scenarios
    if (wr_en && wr_full)  cov_fifo_overflow_attempt  = cov_fifo_overflow_attempt  + 1;
    if (rd_en && rd_empty) cov_fifo_underflow_attempt = cov_fifo_underflow_attempt + 1;
    if (wr_en && rd_en && !wr_full && !rd_empty) cov_fifo_simul_rw = cov_fifo_simul_rw + 1;
    if (almost_full) cov_fifo_almost_full = cov_fifo_almost_full + 1;

    if (count == 0) cov_fifo_count_empty = cov_fifo_count_empty + 1;
    else if (count <= 7) cov_fifo_count_low = cov_fifo_count_low + 1;
    else if (count >= 12 && count <= 15) cov_fifo_count_almost = cov_fifo_count_almost + 1;
    else if (count == FIFO_DEPTH) cov_fifo_count_full = cov_fifo_count_full + 1;
  end

  // =========================================================================
  // Stimulus (constrained-random) - $urandom_range ? ??
  // =========================================================================
  task setup_weights();
    for (int i = 0; i < 4; i++) channel_weight[i] = 8'd2; // ECG
    for (int i = 4; i < 8; i++) channel_weight[i] = 8'd1; // EEG/EMG
    for (int i = 0; i < NUM_CHANNELS; i++) channel_priority[i] = 4'd1;
  endtask

  task drive_random_ready();
    channel_ready = $urandom_range(0, 255);
  endtask

  task drive_all_request();
    channel_ready  = 8'hFF;
    channel_enable = 8'hFF;
  endtask

  task drive_fifo_random();
    wr_en   = $urandom_range(0,1);
    rd_en   = $urandom_range(0,1);
    wr_data = $urandom;
  endtask

  task drive_fifo_overflow();
    rd_en = 0;
    repeat (FIFO_DEPTH + 4) begin
      wr_en = 1;
      wr_data = $urandom;
      @(posedge clk);
    end
    wr_en = 0;
  endtask

  task drive_fifo_underflow();
    wr_en = 0;
    repeat (4) begin
      rd_en = 1;
      @(posedge clk);
    end
    rd_en = 0;
  endtask

  // =========================================================================
  // Main test sequence
  // =========================================================================
  initial begin
    arbiter_mode   = 2'b00;
    channel_enable = 8'hFF;
    channel_ready  = 8'h00;
    channel_urgent = 8'h00;
    adc_busy       = 0;
    channel_accept = 0;
    wr_en = 0; rd_en = 0; wr_data = 0;
    setup_weights();

    sb_arb_checks=0; sb_arb_errors=0;
    sb_fifo_checks=0; sb_fifo_errors=0;
    err_A1=0; err_A2=0; err_A3=0; err_A4=0; err_A5=0;
    err_A6=0; err_A7=0; err_A8=0; err_A9=0; err_A10=0;

    // -------- 1. Reset ? ?? ?? ???? --------
    rst_n = 1;
    @(posedge clk);
    rst_n = 0;
    channel_ready = 8'hFF;
    repeat (2) @(posedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);

    // -------- 2. ? arbiter mode? ?? ????? --------
    for (int m = 0; m < 4; m++) begin
      arbiter_mode = m[1:0];
      repeat (200) begin
        channel_accept = channel_valid && ($urandom_range(0,3) != 0);
        drive_random_ready();
        @(posedge clk);
      end
    end

    // -------- 3. ?? ?? ?? request --------
    repeat (50) begin
      drive_all_request();
      channel_accept = channel_valid;
      @(posedge clk);
    end

    // -------- 4. Arbitration mode ?? (??) --------
    for (int i = 0; i < 40; i++) begin
      arbiter_mode = $urandom_range(0,3);
      drive_random_ready();
      channel_accept = channel_valid;
      @(posedge clk);
    end

    // -------- 4b. Arbitration mode ?? (16?? ?? ?? ??, exhaustive) --------
    // cov_mode_transitions[prev*4 + curr] ? 16/16 ??? ??
    for (int prev_m = 0; prev_m < 4; prev_m++) begin
      arbiter_mode = prev_m[1:0];
      drive_random_ready();
      channel_accept = channel_valid;
      @(posedge clk); // prev_m? ? ?? ???? arbiter_mode_prev? ????? ?

      for (int curr_m = 0; curr_m < 4; curr_m++) begin
        arbiter_mode = curr_m[1:0];
        drive_random_ready();
        channel_accept = channel_valid;
        @(posedge clk); // (prev_m -> curr_m) ??? ? ???? ????

        // ?? curr_m? ?? ?? prev_m?? ???? ?? prev_m->curr_m ??? ??
        arbiter_mode = prev_m[1:0];
        drive_random_ready();
        channel_accept = channel_valid;
        @(posedge clk);
      end
    end

    // -------- 5. FIFO: simultaneous read/write ?? ?? --------
    channel_accept = 0;
    repeat (200) begin
      drive_fifo_random();
      @(posedge clk);
    end

    // -------- 6. FIFO overflow --------
    drive_fifo_overflow();
    @(posedge clk);

    // -------- 7. FIFO underflow --------
    rd_en = 1;
    repeat (FIFO_DEPTH+2) @(posedge clk);
    rd_en = 0;
    @(posedge clk);
    drive_fifo_underflow();

    // -------- ?? ?? --------
    $display("==============================================");
    $display("Arbiter scoreboard: checks=%0d errors=%0d", sb_arb_checks, sb_arb_errors);
    $display("FIFO   scoreboard: checks=%0d errors=%0d", sb_fifo_checks, sb_fifo_errors);
    $display("---- Assertion-equivalent error counts (A1~A10) ----");
    $display("A1=%0d A2=%0d A3=%0d A4=%0d A5=%0d A6=%0d A7=%0d A8=%0d A9=%0d A10=%0d",
             err_A1, err_A2, err_A3, err_A4, err_A5, err_A6, err_A7, err_A8, err_A9, err_A10);
    $display("---- Coverage-equivalent hit counts ----");
    $display("mode RR=%0d Prio=%0d Wgt=%0d Dyn=%0d", cov_mode_rr, cov_mode_prio, cov_mode_wgt, cov_mode_dyn);
    $display("all_channels_req=%0d no_channels_req=%0d reset_with_req=%0d",
             cov_all_channels_req, cov_no_channels_req, cov_reset_with_req);
    $display("fifo overflow_attempt=%0d underflow_attempt=%0d simul_rw=%0d almost_full=%0d",
             cov_fifo_overflow_attempt, cov_fifo_underflow_attempt, cov_fifo_simul_rw, cov_fifo_almost_full);
    $display("fifo count levels: empty=%0d low=%0d almost=%0d full=%0d",
             cov_fifo_count_empty, cov_fifo_count_low, cov_fifo_count_almost, cov_fifo_count_full);
    begin
      int trans_hit;
      trans_hit = 0;
      for (int i = 0; i < 16; i++) if (cov_mode_transitions[i] > 0) trans_hit = trans_hit + 1;
      $display("mode_transition bins hit: %0d / 16", trans_hit);
    end
    $display("==============================================");

    if (sb_arb_errors==0 && sb_fifo_errors==0 &&
        err_A1==0 && err_A2==0 && err_A3==0 && err_A4==0 && err_A5==0 &&
        err_A6==0 && err_A7==0 && err_A8==0 && err_A9==0 && err_A10==0)
      $display("TEST PASSED");
    else
      $display("TEST FAILED");

    $finish;
  end

endmodule



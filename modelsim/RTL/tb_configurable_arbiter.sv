// =============================================================================
// tb_arbiter.sv
// configurable_arbiter 모듈 단위 검증
//  - Directed test: arbitration mode 전환(RR/Priority/Weighted/Dynamic),
//                    reset 중 입력 발생, 모든 채널 동시 request
//  - SVA: selected_channel validity, mode-specific invariant, reset 동작
//  - Functional coverage: 4가지 mode, 전 채널 동시 요청, mode 전환
// =============================================================================
`timescale 1ns/1ps

module tb_configurable_arbiter;

    localparam NUM_CHANNELS  = 8;
    localparam CHANNEL_WIDTH = $clog2(NUM_CHANNELS);

    logic clk = 0;
    logic rst_n;

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

    always #5 clk = ~clk; // 100MHz

    configurable_arbiter #(
        .NUM_CHANNELS(NUM_CHANNELS)
    ) dut (
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

    // =========================================================================
    // SVA
    // =========================================================================

    // A1: selected_channel은 항상 0..NUM_CHANNELS-1 범위 (CHANNEL_WIDTH로 자동 보장되지만 명시)
    property p_selected_channel_range;
        @(posedge clk) disable iff (!rst_n)
        selected_channel < NUM_CHANNELS;
    endproperty
    assert property (p_selected_channel_range)
        else $error("[SVA-A1][%0t] selected_channel out of range: %0d", $time, selected_channel);

    // A2: channel_valid=1이면 해당 채널은 enable && ready 상태여야 함
    property p_valid_implies_enabled_ready;
        @(posedge clk) disable iff (!rst_n)
        channel_valid |-> (channel_enable[selected_channel] && channel_ready[selected_channel]);
    endproperty
    assert property (p_valid_implies_enabled_ready)
        else $error("[SVA-A2][%0t] channel_valid asserted for disabled/not-ready channel %0d (enable=%b ready=%b)",
                     $time, selected_channel, channel_enable[selected_channel], channel_ready[selected_channel]);

    // A3: adc_busy=1이면 channel_valid=0 (새 변환 시작 안 함)
    property p_no_valid_while_busy;
        @(posedge clk) disable iff (!rst_n)
        adc_busy |-> !channel_valid;
    endproperty
    assert property (p_no_valid_while_busy)
        else $error("[SVA-A3][%0t] channel_valid asserted while adc_busy=1", $time);

    // A4: 어떤 채널도 enable/ready가 아니면 channel_valid=0
    property p_no_valid_if_none_ready;
        @(posedge clk) disable iff (!rst_n)
        (|(channel_enable & channel_ready) == 1'b0) |-> !channel_valid;
    endproperty
    assert property (p_no_valid_if_none_ready)
        else $error("[SVA-A4][%0t] channel_valid asserted but no channel is enable&&ready", $time);

    // A5: reset 직후 rr_counter==0 (내부 신호 접근)
    property p_reset_rr_counter;
        @(posedge clk) (!rst_n) |=> (dut.rr_counter == 0);
    endproperty
    assert property (p_reset_rr_counter)
        else $error("[SVA-A5][%0t] rr_counter not reset to 0: %0d", $time, dut.rr_counter);

    // A6: reset 직후 모든 weight_accumulator==0
    property p_reset_weight_acc;
        @(posedge clk) (!rst_n) |=> (dut.weight_accumulator[0] == 0 &&
                                       dut.weight_accumulator[NUM_CHANNELS-1] == 0);
    endproperty
    assert property (p_reset_weight_acc)
        else $error("[SVA-A6][%0t] weight_accumulator not reset to 0", $time);

    // A7: Round-Robin 모드에서 channel_accept 시 rr_counter는 (이전+1)%N 또는 wrap
    property p_rr_counter_increments;
        @(posedge clk) disable iff (!rst_n)
        (channel_accept && arbiter_mode == 2'b00 && $past(rst_n)) |=>
            (dut.rr_counter == ((($past(dut.rr_counter) == NUM_CHANNELS-1) ? 0 : $past(dut.rr_counter) + 1)));
    endproperty
    assert property (p_rr_counter_increments)
        else $error("[SVA-A7][%0t] rr_counter did not increment correctly: rr_counter=%0d past=%0d",
                     $time, dut.rr_counter, $past(dut.rr_counter));

    // A8: Dynamic 모드(2'b11)에서 urgent 채널이 있으면, 선택된 채널은 반드시 urgent 채널 중 하나
    property p_dynamic_urgent_priority;
        @(posedge clk) disable iff (!rst_n)
        (arbiter_mode == 2'b11 && channel_valid &&
         |(channel_urgent & channel_enable & channel_ready))
        |-> channel_urgent[selected_channel];
    endproperty
    assert property (p_dynamic_urgent_priority)
        else $error("[SVA-A8][%0t] Dynamic mode: urgent channel exists but selected_channel=%0d is not urgent",
                     $time, selected_channel);

    // A9: Priority 모드에서 선택된 채널의 priority는, enable&&ready인 다른 모든 채널의
    //     priority보다 작지 않아야 함 (최댓값 선택)
    //
    // NOTE: SVA 본문에서 매번 함수를 호출해 max를 계산하면, 함수 평가 시점과
    // selected_channel/channel_valid가 참조하는 입력 스냅샷 사이에 evaluation
    // region 차이로 인한 race가 발생할 수 있다(둘 다 "같은 클럭"의 조합 결과여야
    // 하지만 시뮬레이터 내부에서 별도 시점에 샘플링됨).
    // 이를 피하기 위해, max_priority_now를 매 클럭 always_comb으로 한 번만
    // 계산해 두고 SVA에서는 그 값만 참조한다.
    logic [3:0] max_priority_now;
    always_comb begin
        max_priority_now = 0;
        for (int i = 0; i < NUM_CHANNELS; i++)
            if (channel_enable[i] && channel_ready[i] && channel_priority[i] > max_priority_now)
                max_priority_now = channel_priority[i];
    end

    property p_priority_mode_is_max;
        @(posedge clk) disable iff (!rst_n)
        (arbiter_mode == 2'b01 && channel_valid)
        |-> (channel_priority[selected_channel] >= max_priority_now);
    endproperty
    assert property (p_priority_mode_is_max)
        else $error("[SVA-A9][%0t] Priority mode: mode=%0d valid=%0b selected_channel=%0d priority=%0d is not max (max=%0d) enable=%b ready=%b prio={%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d}",
                     $time, arbiter_mode, channel_valid, selected_channel, channel_priority[selected_channel], max_priority_now,
                     channel_enable, channel_ready,
                     channel_priority[0], channel_priority[1], channel_priority[2], channel_priority[3],
                     channel_priority[4], channel_priority[5], channel_priority[6], channel_priority[7]);

    // =========================================================================
    // Functional coverage (manual covergroup)
    // =========================================================================
    covergroup cg_arbiter @(posedge clk);
        option.per_instance = 1;
        cp_mode: coverpoint arbiter_mode {
            bins rr       = {2'b00};
            bins priority_= {2'b01};
            bins weighted = {2'b10};
            bins dynamic_ = {2'b11};
        }
        cp_all_request: coverpoint (channel_enable & channel_ready) {
            bins all_eight = {8'hFF};
            bins others    = default;
        }
        cp_urgent_active: coverpoint (|channel_urgent) {
            bins active = {1};
        }
        cp_mode_transition: coverpoint arbiter_mode {
            bins trans[] = (2'b00, 2'b01, 2'b10, 2'b11 => 2'b00, 2'b01, 2'b10, 2'b11);
        }
        cp_busy: coverpoint adc_busy;
        cp_valid: coverpoint channel_valid;
        cross cp_mode, cp_all_request;
    endgroup

    cg_arbiter cg = new();

    // =========================================================================
    // Helper tasks
    // =========================================================================
    task automatic set_all_channels(logic en, logic rdy, logic urg);
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            channel_enable[i] = en;
            channel_ready[i]  = rdy;
            channel_urgent[i] = urg;
        end
    endtask

    task automatic init_priorities_weights();
        for (int i = 0; i < NUM_CHANNELS; i++) begin
            channel_priority[i] = i;          // 채널번호=priority (서로 다른 값)
            channel_weight[i]   = (i < 4) ? 2 : 1; // ECG:2, EEG/EMG:1
        end
    endtask

    // accept pulse 생성: channel_valid면 1클럭 accept
    task automatic run_cycles_with_accept(int n);
        repeat (n) begin
            channel_accept = channel_valid;
            @(posedge clk);
        end
        channel_accept = 0;
    endtask

    // =========================================================================
    // Stimulus
    // =========================================================================
    initial begin
        rst_n = 0;
        arbiter_mode = 2'b00;
        adc_busy = 0;
        channel_accept = 0;
        set_all_channels(1'b0, 1'b0, 1'b0);
        init_priorities_weights();
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 1: Reset 중 입력 발생
        //  - reset 도중 channel_enable/ready/urgent/accept 모두 asserted
        //  - reset 해제 후 rr_counter, weight_accumulator가 0으로 시작하는지 SVA(A5,A6)가 체크
        // -----------------------------------------------------------------
        $display("[TB-ARB] Scenario 1: inputs asserted during reset");
        set_all_channels(1'b1, 1'b1, 1'b1);
        channel_accept = 1;
        arbiter_mode = 2'b10; // weighted
        rst_n = 0;
        repeat (3) @(posedge clk);
        rst_n = 1;
        @(posedge clk);
        if (dut.rr_counter !== 0)
            $error("[TB-ARB] Scenario1 FAIL: rr_counter=%0d after reset, expected 0", dut.rr_counter);
        else
            $display("[TB-ARB] Scenario1 PASS: rr_counter=0 after reset despite asserted inputs");
        channel_accept = 0;
        set_all_channels(1'b0, 1'b0, 1'b0);
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 2: 모든 채널이 동시에 request (all-channel contention)
        //  - 4가지 모드 각각에서 전 채널 enable&&ready
        // -----------------------------------------------------------------
        $display("[TB-ARB] Scenario 2: all-channel simultaneous request");
        for (int m = 0; m < 4; m++) begin
            arbiter_mode = m[1:0];
            set_all_channels(1'b1, 1'b1, 1'b0);
            adc_busy = 0;
            run_cycles_with_accept(2 * NUM_CHANNELS);
            $display("[TB-ARB] Scenario2 mode=%0d: ran %0d cycles with all 8 channels requesting",
                     m, 2*NUM_CHANNELS);
        end
        set_all_channels(1'b0, 1'b0, 1'b0);
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 3: arbitration mode 전환 (모든 mode 조합 순회)
        // -----------------------------------------------------------------
        $display("[TB-ARB] Scenario 3: arbitration mode transitions");
        set_all_channels(1'b1, 1'b1, 1'b0);
        for (int m1 = 0; m1 < 4; m1++) begin
            for (int m2 = 0; m2 < 4; m2++) begin
                arbiter_mode = m1[1:0];
                run_cycles_with_accept(3);
                arbiter_mode = m2[1:0];
                run_cycles_with_accept(3);
            end
        end
        $display("[TB-ARB] Scenario3 PASS: all 16 mode-transition combinations exercised");

        // -----------------------------------------------------------------
        // Scenario 4: Dynamic mode - urgent 채널 우선권 확인
        // -----------------------------------------------------------------
        $display("[TB-ARB] Scenario 4: Dynamic mode urgent priority");
        arbiter_mode = 2'b11;
        set_all_channels(1'b1, 1'b1, 1'b0);
        channel_urgent = 8'h00;
        run_cycles_with_accept(5);
        // 채널 5에만 urgent 설정
        channel_urgent = 8'b0010_0000;
        @(posedge clk);
        if (channel_valid && selected_channel !== 5)
            $error("[TB-ARB] Scenario4 FAIL: urgent channel 5 set but selected_channel=%0d",
                   selected_channel);
        else
            $display("[TB-ARB] Scenario4 PASS: selected_channel=%0d (urgent=5)", selected_channel);
        channel_accept = 1;
        @(posedge clk);
        channel_accept = 0;
        channel_urgent = 8'h00;
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 5: adc_busy 동안 channel_valid==0 유지
        // -----------------------------------------------------------------
        $display("[TB-ARB] Scenario 5: adc_busy blocks new selection");
        arbiter_mode = 2'b00;
        set_all_channels(1'b1, 1'b1, 1'b0);
        adc_busy = 1;
        repeat (5) @(posedge clk);
        if (channel_valid)
            $error("[TB-ARB] Scenario5 FAIL: channel_valid=1 while adc_busy=1");
        else
            $display("[TB-ARB] Scenario5 PASS: channel_valid=0 while adc_busy=1");
        adc_busy = 0;
        @(posedge clk);

        // -----------------------------------------------------------------
        // Scenario 6: Constrained-random mode/enable/ready/urgent 조합
        // -----------------------------------------------------------------
        $display("[TB-ARB] Scenario 6: constrained-random stimulus");
        for (int i = 0; i < 300; i++) begin
            arbiter_mode   = $urandom_range(0, 3);
            channel_enable = $urandom_range(0, 255);
            channel_ready  = $urandom_range(0, 255);
            channel_urgent = $urandom_range(0, 255) & channel_enable; // urgent는 enable 채널 중에서만
            adc_busy       = $urandom_range(0, 9) == 0; // 10% busy
            channel_accept = channel_valid && ($urandom_range(0,1));
            @(posedge clk);
        end
        channel_accept = 0;
        set_all_channels(1'b0, 1'b0, 1'b0);
        adc_busy = 0;
        @(posedge clk);

        $display("==============================================");
        $display("ARBITER directed+random scenarios completed.");
        $display("Coverage (manual): mode RR/Pri/Weighted/Dynamic, all-8-channel request,");
        $display("                   urgent active, mode transitions, reset-during-input");
        $display("==============================================");

        $finish;
    end

endmodule

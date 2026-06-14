// =============================================================================
// tb_top_spi.sv
//   - high_speed_daq_controller (top-level) 검증
//   - SPI write (config register 설정) + SPI read (FIFO -> MISO) 트랜잭션
//   - ADC behavioral model
//   - CDC(v1, no-sync) 동작을 waveform으로 직접 확인하기 위한 목적
//
//   설계 결정 (예측 가능하게 만들기 위함):
//     - SPI write로 config_registers[1]=8'h01 (CH0만 enable)
//                    config_registers[2]=2'b00 (Round-Robin)
//     -> CH0만 항상 선택됨 (arbiter 단순화)
//     - test_mode=0 : channel_ready_internal = channel_enable, adc_busy=0 고정
//     - ADC model: adc_start_conv 받으면 1클럭 뒤 adc_conv_done=1,
//                   adc_data = 증가하는 카운터 값
//     => FIFO에 들어가는 값 = {1'b0, 3'b000, adc_counter} (하위 12bit가 카운터값)
//     - SPI read로 FIFO에서 순서대로 꺼내 expected 값과 비교
// =============================================================================

`timescale 1ns/1ps

module tb_top_spi;

  localparam NUM_CHANNELS = 8;
  localparam ADC_WIDTH    = 12;
  localparam CHANNEL_WIDTH = $clog2(NUM_CHANNELS);

  // ---------------------------------------------------------------------
  // Clocks / Reset
  // ---------------------------------------------------------------------
  logic clk;
  logic rst_n;

  initial clk = 0;
  always #5 clk = ~clk; // 100MHz (10ns period)

  // spi_sclk: clk보다 훨씬 느림 (v1 CDC 가정: spi_sclk << clk)
  // period = 1000ns -> clk(10ns)의 100배
  logic spi_sclk;
  initial spi_sclk = 0;
  always #500 spi_sclk = ~spi_sclk;

  // ---------------------------------------------------------------------
  // DUT I/O
  // ---------------------------------------------------------------------
  logic spi_mosi, spi_miso, spi_cs_n;
  logic adc_start_conv, adc_conv_done, adc_busy;
  logic [CHANNEL_WIDTH-1:0] adc_channel_sel;
  logic [ADC_WIDTH-1:0] adc_data;
  logic interrupt;
  logic [7:0] status_leds;
  logic test_mode;
  logic [NUM_CHANNELS-1:0] test_channel_ready, test_adc_busy_unused;

  high_speed_daq_controller #(
    .NUM_CHANNELS(NUM_CHANNELS), .ADC_WIDTH(ADC_WIDTH), .FIFO_DEPTH(16)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
    .adc_start_conv(adc_start_conv), .adc_channel_sel(adc_channel_sel),
    .adc_conv_done(adc_conv_done), .adc_data(adc_data), .adc_busy(adc_busy),
    .interrupt(interrupt), .status_leds(status_leds),
    .test_mode(test_mode), .test_channel_ready(test_channel_ready), .test_adc_busy(1'b0)
  );

  // ---------------------------------------------------------------------
  // ADC Behavioral Model
  //   - adc_start_conv 다음 클럭에 adc_conv_done=1, adc_data=counter
  //   - counter는 매 변환마다 +1
  // ---------------------------------------------------------------------
  logic [ADC_WIDTH-1:0] adc_counter;
  logic [15:0] expected_fifo_q [0:1023];
  int exp_tail;
  int exp_head = 0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      adc_conv_done <= 0;
      adc_data <= 0;
      adc_busy <= 0;
      adc_counter <= 0;
      exp_tail <= 0;
    end else begin
      adc_conv_done <= 1'b0;
      if (adc_start_conv) begin
        adc_conv_done <= 1'b1;
        adc_data      <= adc_counter;

        // RTL이 locked_channel(=adc_start_conv 시점의 selected_channel)을
        // fifo_wr_data의 channel 필드로 사용하므로, 모델도 동일 시점에 기록
        expected_fifo_q[exp_tail] <= {1'b0, adc_channel_sel, adc_counter};
        exp_tail <= exp_tail + 1;

        adc_counter   <= adc_counter + 1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // test_mode 설정: channel_ready_internal = channel_enable (항상 ready)
  // ---------------------------------------------------------------------
  assign test_mode = 1'b0;
  assign test_channel_ready = '0; // test_mode=0이므로 미사용

  // =========================================================================
  // SPI Driver Tasks
  //   command byte: [7]=R/W (1=read,0=write), [6:0]=reserved(0)
  //   address: 16bit (register index << 2, 즉 reg_addr_cdc[7:2]=reg_index)
  //   write data: 32bit
  //   read data : 16bit (FIFO 데이터, BYTE0=상위8, BYTE1=하위8)
  //
  //   DUT가 posedge spi_sclk에서 spi_mosi를 샘플링하므로,
  //   negedge spi_sclk에서 spi_mosi를 세팅한다 (setup time 확보)
  // =========================================================================

  // CS를 명시적으로 deassert(1)했다가 다시 assert(0)하면서,
  // 2-flop spi_cs_n_sync가 1->0으로 정상 settle할 시간을 줌.
  // settle 중의 edge들은 spi_cs_n_sync==1인 동안 bit_counter=0/shift_reg=0으로
  // 유지되므로(DUT의 if(spi_cs_n_sync) 분기) 안전하게 "흡수"된다.
  task spi_begin_transaction();
    spi_mosi = 0;
    spi_cs_n = 1;
    repeat (3) @(posedge spi_sclk); // cs_n_sync -> 1, state/bit_counter reset 확정
    spi_cs_n = 0;
    repeat (2) @(posedge spi_sclk); // cs_n_sync -> 0 으로 정확히 settle (bit_counter=0 유지)
  endtask

  task spi_send_bit(input logic b);
    @(negedge spi_sclk);
    spi_mosi = b;
    @(posedge spi_sclk); // DUT가 이 edge에서 샘플링
  endtask

  task spi_send_byte(input logic [7:0] data);
    for (int i = 7; i >= 0; i--) spi_send_bit(data[i]);
  endtask

  // SPI Write: register write
  task spi_write_reg(input logic [15:0] addr, input logic [31:0] wdata);
    spi_begin_transaction();
    spi_send_byte(8'h00); // CMD: write (MSB=0)
    spi_send_byte(addr[15:8]);
    spi_send_byte(addr[7:0]);
    spi_send_byte(wdata[31:24]);
    spi_send_byte(wdata[23:16]);
    spi_send_byte(wdata[15:8]);
    spi_send_byte(wdata[7:0]);
    @(negedge spi_sclk);
    spi_cs_n = 1; // CS deassert -> write_pending 처리 트리거
    repeat (3) @(posedge spi_sclk); // ack handshake 진행 시간 확보
  endtask

  // SPI Read: FIFO read (1개 word = 16bit, 2바이트)
  task spi_read_fifo(output logic [15:0] rdata);
    logic [7:0] byte0, byte1;
    spi_begin_transaction();
    spi_send_byte(8'h80); // CMD: read (MSB=1)
    spi_send_byte(8'h00); // addr high (read에서는 의미 없음, 0)
    spi_send_byte(8'h00); // addr low -> 이 시점에 fifo_rd_en_spi pulse 발생

    // DATA_BYTE0/1 구간: MISO에서 읽어옴 (MOSI는 don't-care, 0으로 둠)
    byte0 = 8'h00;
    byte1 = 8'h00;
    for (int i = 7; i >= 0; i--) begin
      @(negedge spi_sclk);
      spi_mosi = 1'b0;
      @(posedge spi_sclk);
      byte0[i] = spi_miso;
    end
    for (int i = 7; i >= 0; i--) begin
      @(negedge spi_sclk);
      spi_mosi = 1'b0;
      @(posedge spi_sclk);
      byte1[i] = spi_miso;
    end

    @(negedge spi_sclk);
    spi_cs_n = 1;
    rdata = {byte0, byte1};
  endtask

  // =========================================================================
  // DEBUG MONITOR: SPI read CDC 관련 신호 변화를 추적
  // =========================================================================
  always @(posedge spi_sclk) begin
    if (dut.fifo_rd_en_spi)
      $display("[DBG-SPI][%0t] fifo_rd_en_spi=1 spi_read_mode=%0b spi_shift_reg=%h bit_counter=%0d spi_state=%0d",
               $time, dut.spi_read_mode, dut.spi_shift_reg, dut.bit_counter, dut.spi_state);
    if (dut.fifo_rd_en_spi) // edge에서 latch되는 시점도 같이 표시
      $display("[DBG-SPI][%0t] (latched next) fifo_read_data_latched(prev val)=%h", $time, dut.fifo_read_data_latched);
  end

  always @(posedge clk) begin
    if (dut.fifo_rd_en)
      $display("[DBG-CLK][%0t] fifo_rd_en pulse! fifo_rd_data(before pop)=%h fifo_count(before)=%0d -> fifo_read_data_latched will become this value",
               $time, dut.fifo_rd_data, dut.fifo_count);
  end

  always @(posedge spi_sclk) begin
    // SPI_CMD 완료 시점(spi_read_mode 갱신 직후 edge)에 값 출력
    if (dut.spi_state == dut.SPI_ADDR_HIGH)
      $display("[DBG-SPI][%0t] entered SPI_ADDR_HIGH, spi_read_mode=%0b spi_shift_reg=%h", $time, dut.spi_read_mode, dut.spi_shift_reg);
  end


  int read_idx;
  int sb_top_checks, sb_top_errors;

  initial begin
    spi_cs_n = 1;
    spi_mosi = 0;
    rst_n = 0;
    read_idx = 0;
    sb_top_checks = 0; sb_top_errors = 0;

    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (5) @(posedge clk);

    // -------------------------------------------------------------------
    // 1. SPI write: CH0만 enable, RR mode
    //    config_registers[1] -> addr = 1<<2 = 4  (reg_addr_cdc[7:2]=1)
    //    config_registers[2] -> addr = 2<<2 = 8
    // -------------------------------------------------------------------
    spi_write_reg(16'h0004, 32'h0000_0001); // channel_enable = 8'h01 (CH0만)
    spi_write_reg(16'h0008, 32'h0000_0000); // arbiter_mode   = 0 (RR)

    // -------------------------------------------------------------------
    // 2. FIFO에 데이터가 쌓일 시간을 줌
    //    CH0가 RR로 계속 선택 -> ADC FSM 1회전마다 1 sample
    //    FSM 1회전 ~ settling(8) + start(1) + wait(1) + capture(1) + next(1) ~ 12 cycles
    //    여유있게 N개 샘플 만들기 위해 충분히 대기
    // -------------------------------------------------------------------
    repeat (200) @(posedge clk); // 약 10+ samples 생성 예상

    // -------------------------------------------------------------------
    // 2b. FIFO 채우기 완료 후, read 도중 새 샘플이 추가/대체되지 않도록
    //     channel_enable=0으로 ADC 샘플 생성을 멈춤
    //     (SPI read 1회 = 약 4500 clk이 걸려, 막아두지 않으면 read 도중에도
    //      FIFO가 계속 refill되어 idx 기반 expected 비교가 무의미해짐)
    // -------------------------------------------------------------------
    spi_write_reg(16'h0004, 32'h0000_0000); // channel_enable = 0 (모든 채널 비활성)
    repeat (20) @(posedge clk); // 진행 중이던 FSM이 안전하게 멈출 시간

    // -------------------------------------------------------------------
    // 3. SPI read로 FIFO 데이터 꺼내서 expected와 비교
    //    exp_tail 까지 쌓인 만큼 읽음
    // -------------------------------------------------------------------
    begin
      int n_expected;
      n_expected = exp_tail;
      if (n_expected > 16) n_expected = 16; // FIFO_DEPTH=16 -> wr_full 이후 샘플은 버려짐
      $display("Expected FIFO entries available (total generated=%0d, comparing first %0d): %0d",
               exp_tail, n_expected, n_expected);

      for (int i = 0; i < n_expected; i++) begin
        logic [15:0] rdata;
        spi_read_fifo(rdata);
        sb_top_checks++;
        if (rdata !== expected_fifo_q[exp_head]) begin
          sb_top_errors++;
          $display("[SB-TOP][%0t] SPI-READ MISMATCH idx=%0d expected=%h got=%h",
                   $time, i, expected_fifo_q[exp_head], rdata);
        end else begin
          $display("[SB-TOP][%0t] SPI-READ OK idx=%0d data=%h", $time, i, rdata);
        end
        exp_head++;
      end
    end

    $display("==============================================");
    $display("TOP scoreboard: checks=%0d errors=%0d", sb_top_checks, sb_top_errors);
    if (sb_top_errors == 0) $display("TOP TEST PASSED");
    else $display("TOP TEST FAILED");
    $display("==============================================");

    $finish;
  end

endmodule



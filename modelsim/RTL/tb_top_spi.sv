// =============================================================================
// tb_top_spi.sv
// - high_speed_daq_controller (top-level) verification
// - SPI write (config register setting) + SPI read (FIFO -> MISO) transaction
// - ADC behavioral model
// - Purpose: To directly verify CDC (v1, no-sync) operation via waveform
// Design decision (to make it predictable):
// - config_registers[1]=8'h01 via SPI write (enable only CH0)
// config_registers[2]=2'b00 (Round-Robin)
// -> Only CH0 is always selected (arbiter simplification)
// - test_mode=0 : channel_ready_internal = channel_enable, adc_busy=0 fixed
// - ADC model: when adc_start_conv is received, adc_conv_done=1 after 1 clock cycle,
// adc_data = incrementing counter value
// => Value entering FIFO = {1'b0, 3'b000, adc_counter} (lower 12 bits are counter value)
// - Read values ​​sequentially from FIFO using SPI read and compare with expected value
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

  // MAC array interface signals
  logic signed [7:0] mac_weight_tb [0:NUM_CHANNELS-1][0:3];
  logic [31:0]       mac_threshold_tb;
  logic              mac_alert_tb;

  high_speed_daq_controller #(
    .NUM_CHANNELS(NUM_CHANNELS), .ADC_WIDTH(ADC_WIDTH), .FIFO_DEPTH(16)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .spi_sclk(spi_sclk), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
    .adc_start_conv(adc_start_conv), .adc_channel_sel(adc_channel_sel),
    .adc_conv_done(adc_conv_done), .adc_data(adc_data), .adc_busy(adc_busy),
    .interrupt(interrupt), .status_leds(status_leds),
    .mac_weight(mac_weight_tb),
    .mac_threshold(mac_threshold_tb),
    .mac_alert(mac_alert_tb),
    .test_mode(test_mode), .test_channel_ready(test_channel_ready), .test_adc_busy(1'b0)
  );

  // ---------------------------------------------------------------------
  // ADC Behavioral Model
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

        expected_fifo_q[exp_tail] <= {1'b0, adc_channel_sel, adc_counter};
        exp_tail <= exp_tail + 1;

        adc_counter   <= adc_counter + 1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // test_mode: channel_ready_internal = channel_enable
  // ---------------------------------------------------------------------
  assign test_mode = 1'b0;
  assign test_channel_ready = '0;
  
  // ---------------------------------------------------------------------
  // MAC array
  // Set MAC array initial values
  // mac_weight: Use the same value (10) for all channels/taps (for simple verification)
  // mac_threshold: Set to half the maximum dot product value (4095 * 10 * 4 = 163800)
  // → No alert is triggered for normal signals (adc_counter=0..15)
  // Can check if an alert is triggered as the counter increases
  // ---------------------------------------------------------------------
  initial begin
    mac_threshold_tb = 32'd50000;
    for (int c = 0; c < NUM_CHANNELS; c++)
      for (int k = 0; k < 4; k++)
        mac_weight_tb[c][k] = 8'd10;
  end

  always @(posedge clk) begin
    if (mac_alert_tb)
      $display("[MAC-ALERT][%0t] mac_alert asserted (threshold=%0d)",
               $time, mac_threshold_tb);
  end
  
  // =========================================================================
  // SPI Driver Tasks
  //   command byte: [7]=R/W (1=read,0=write), [6:0]=reserved(0)
  //   address: 16bit (register index << 2, 즉 reg_addr_cdc[7:2]=reg_index)
  //   write data: 32bit
  //   read data : 16bit (FIFO data, BYTE0=upper 8, BYTE1=under8)
  // =========================================================================

  task spi_begin_transaction();
    spi_mosi = 0;
    spi_cs_n = 1;
    repeat (3) @(posedge spi_sclk); // cs_n_sync -> 1, state/bit_counter reset
    spi_cs_n = 0;
    repeat (2) @(posedge spi_sclk); // cs_n_sync -> 0
  endtask

  task spi_send_bit(input logic b);
    @(negedge spi_sclk);
    spi_mosi = b;
    @(posedge spi_sclk);
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
    spi_cs_n = 1;
    repeat (3) @(posedge spi_sclk);
  endtask

  // SPI Read: FIFO read
  task spi_read_fifo(output logic [15:0] rdata);
    logic [7:0] byte0, byte1;
    spi_begin_transaction();
    spi_send_byte(8'h80); // CMD: read (MSB=1)
    spi_send_byte(8'h00); // addr high
    spi_send_byte(8'h00); // addr low -> fifo_rd_en_spi pulse 

    // DATA_BYTE0/1: MISO
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
  // DEBUG MONITOR: SPI read CDC
  // =========================================================================
  always @(posedge spi_sclk) begin
    if (dut.fifo_rd_en_spi)
      $display("[DBG-SPI][%0t] fifo_rd_en_spi=1 spi_read_mode=%0b spi_shift_reg=%h bit_counter=%0d spi_state=%0d",
               $time, dut.spi_read_mode, dut.spi_shift_reg, dut.bit_counter, dut.spi_state);
    if (dut.fifo_rd_en_spi)
      $display("[DBG-SPI][%0t] (latched next) fifo_read_data_latched(prev val)=%h", $time, dut.fifo_read_data_latched);
  end

  always @(posedge clk) begin
    if (dut.fifo_rd_en)
      $display("[DBG-CLK][%0t] fifo_rd_en pulse! fifo_rd_data(before pop)=%h fifo_count(before)=%0d -> fifo_read_data_latched will become this value",
               $time, dut.fifo_rd_data, dut.fifo_count);
  end

  always @(posedge spi_sclk) begin
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
    // 1. SPI write: CH0 enable, RR mode
    //    config_registers[1] -> addr = 1<<2 = 4  (reg_addr_cdc[7:2]=1)
    //    config_registers[2] -> addr = 2<<2 = 8
    // -------------------------------------------------------------------
    spi_write_reg(16'h0004, 32'h0000_0001); // channel_enable = 8'h01 (CH0)
    spi_write_reg(16'h0008, 32'h0000_0000); // arbiter_mode   = 0 (RR)

    // -------------------------------------------------------------------
    // 2. Allow time for data to accumulate in the FIFO
    // CH0 continuously selects RR -> 1 sample per ADC FSM rotation
    // 1 FSM rotation ~ settling(8) + start(1) + wait(1) + capture(1) + next(1) ~ 12 cycles
    // Wait sufficiently to generate N samples with ample time
    // -------------------------------------------------------------------
    repeat (200) @(posedge clk); // 약 10+ samples 생성 예상

    // -------------------------------------------------------------------
    // 2b. After the FIFO is full, to prevent new samples from being added or replaced during a read
    // Stop ADC sample generation by setting channel_enable=0
    // (One SPI read takes approximately 4500 clock cycles; if this is not blocked,
    // the FIFO will continue to refill even during a read, rendering index-based expected comparisons meaningless)
    // -------------------------------------------------------------------
    spi_write_reg(16'h0004, 32'h0000_0000); // channel_enable = 0
    repeat (20) @(posedge clk);

    // -------------------------------------------------------------------
    // 3. Retrieve FIFO data using SPI read and compare with expected
    // Read up to exp_tail
    // -------------------------------------------------------------------
    begin
      int n_expected;
      n_expected = exp_tail;
      if (n_expected > 16) n_expected = 16; // FIFO_DEPTH=16 -> wr_full
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



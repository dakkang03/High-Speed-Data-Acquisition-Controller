// =============================================================================
// high_speed_daq_controller (v2)
//  - 변경점:
//    1) NUM_CHANNELS=8, FIFO_DEPTH=16로 통일
//    2) Reset 기본값: arbiter_mode=Weighted(2'b10),
//       channel_weight: ECG(ch0-3)=2, EEG/EMG(ch4-7)=1
//    3) SPI 양방향(v1, CDC 생략 단순화):
//       - command byte의 MSB(spi_shift_reg[6])로 read/write 구분
//       - read인 경우 ADDR_LOW 완료 시점에 fifo_rd_en pulse 발생
//         (clk와 spi_sclk이 비동기이므로 v1에서는 "충분히 느린 spi_sclk" 가정하에
//          fifo_rd_data를 비동기적으로 직접 latch - 추후 CDC 추가 예정)
// =============================================================================
module high_speed_daq_controller #(
    parameter NUM_CHANNELS = 8,
    parameter ADC_WIDTH = 12,
    parameter FIFO_DEPTH = 16,
    parameter CHANNEL_WIDTH = $clog2(NUM_CHANNELS),
    parameter TIMESTAMP_WIDTH = 32
)(
    input  logic clk,
    input  logic rst_n,

    input  logic spi_sclk,
    input  logic spi_mosi,
    output logic spi_miso,
    input  logic spi_cs_n,

    output logic adc_start_conv,
    output logic [CHANNEL_WIDTH-1:0] adc_channel_sel,
    input  logic adc_conv_done,
    input  logic [ADC_WIDTH-1:0] adc_data,
    input  logic adc_busy,

    output logic interrupt,
    output logic [7:0] status_leds,

    input  logic test_mode,
    input  logic [NUM_CHANNELS-1:0] test_channel_ready,
    input  logic test_adc_busy
);

// =============================================================================
// Signal Declarations
// =============================================================================
logic [15:0] reg_addr_cdc;
logic [31:0] reg_wdata_cdc;
logic reg_write;
logic reg_write_pulse;
logic [3:0] write_hold_counter;

logic [31:0] config_registers [32-1:0];

typedef enum logic [2:0] {
    SPI_IDLE, SPI_CMD, SPI_ADDR_HIGH, SPI_ADDR_LOW,
    SPI_DATA_BYTE0, SPI_DATA_BYTE1, SPI_DATA_BYTE2, SPI_DATA_BYTE3
} spi_state_t;

spi_state_t spi_state, spi_next_state;
logic [2:0] bit_counter;
logic [7:0] spi_shift_reg;
logic spi_cs_n_sync, spi_cs_n_meta;

logic [15:0] reg_addr_spi;
logic [31:0] reg_wdata_spi;

logic req_valid_spi;
logic ack_spi_sync1, ack_spi_sync2;
logic write_pending;
logic spi_cs_n_sync_prev;

logic req_valid_clk_sync1, req_valid_clk_sync2;
logic req_valid_clk_prev;
logic req_rising_edge_clk;
logic ack_clk;
logic ack_clk_sync1, ack_clk_sync2;

// --- SPI READ (v1, simplified, no CDC) ---
logic spi_read_mode;          // command MSB: 1=read, 0=write
logic [15:0] fifo_read_buffer; // latched fifo_rd_data, shifted out via MISO
logic fifo_rd_en_spi;          // pulse: request FIFO read (async-ish, v1)

// clk 도메인: fifo_rd_en_spi(spi_sclk, level) -> fifo_rd_en(clk, 1-cycle pulse)
logic rd_req_sync1, rd_req_sync2, rd_req_sync2_d;
logic [15:0] fifo_read_data_latched;

logic [CHANNEL_WIDTH-1:0] selected_channel;
logic channel_valid, channel_accept;
logic [NUM_CHANNELS-1:0] channel_enable, channel_ready, channel_urgent;
logic [1:0] arbiter_mode;
logic [3:0] channel_priority [NUM_CHANNELS-1:0];
logic [7:0] channel_weight [NUM_CHANNELS-1:0];

logic [15:0] fifo_wr_data;
logic fifo_wr_en, fifo_rd_en;
logic [15:0] fifo_rd_data;
logic fifo_full, fifo_empty, fifo_almost_full;
logic [4:0] fifo_count;

logic trigger_detected, trigger_valid;
logic [7:0] trigger_confidence;
logic [15:0] trigger_metadata;
logic [31:0] trigger_config [8-1:0];

logic [31:0] throughput_sps, avg_latency_ns, max_latency_ns;
logic [7:0] fifo_utilization_pct;
logic [15:0] trigger_rate_ppm;
logic [7:0] warning_flags;
logic [31:0] debug_counters;

typedef enum logic [2:0] {
    IDLE, SELECT_CHANNEL, START_CONVERSION, WAIT_CONVERSION, CAPTURE_DATA, NEXT_CHANNEL
} adc_state_t;

adc_state_t current_state, next_state;
logic [7:0] settling_counter;
logic sample_ready;
logic [ADC_WIDTH-1:0] captured_data;
logic [TIMESTAMP_WIDTH-1:0] global_timestamp;
// START_CONVERSION 시점의 selected_channel을 latch.
// weighted mode에서는 weight_accumulator가 매 클럭 갱신되어
// START_CONVERSION ~ CAPTURE_DATA(2클럭) 사이 selected_channel이 바뀔 수 있음.
// fifo_wr_data의 channel 필드는 "데이터를 실제로 샘플링한 채널"이어야 하므로,
// 변환 시작 시점의 채널을 latch해서 CAPTURE_DATA까지 들고 간다.
logic [CHANNEL_WIDTH-1:0] locked_channel;

logic adc_busy_internal;
logic [NUM_CHANNELS-1:0] channel_ready_internal;

assign adc_busy_internal = test_mode ? test_adc_busy : adc_busy;
assign channel_ready_internal = test_mode ? test_channel_ready : channel_enable;

// =============================================================================
// SPI CS Synchronizer
// =============================================================================
always_ff @(posedge spi_sclk or negedge rst_n) begin
    if (!rst_n) begin
        spi_cs_n_meta <= 1'b1;
        spi_cs_n_sync <= 1'b1;
    end else begin
        spi_cs_n_meta <= spi_cs_n;
        spi_cs_n_sync <= spi_cs_n_meta;
    end
end

// =============================================================================
// SPI Slave Interface (v2: read/write 구분 추가)
// =============================================================================
always_ff @(posedge spi_sclk or negedge rst_n) begin
    if (!rst_n) begin
        bit_counter <= 0;
        spi_shift_reg <= 0;
        spi_state <= SPI_IDLE;
        reg_addr_spi <= 0;
        reg_wdata_spi <= 0;
        req_valid_spi <= 1'b0;
        ack_spi_sync1 <= 1'b0;
        ack_spi_sync2 <= 1'b0;
        write_pending <= 1'b0;
        spi_cs_n_sync_prev <= 1'b1;
        spi_read_mode <= 1'b0;
        fifo_rd_en_spi <= 1'b0;
        fifo_read_buffer <= 16'h0;
    end else begin
        ack_spi_sync1 <= ack_clk_sync2;
        ack_spi_sync2 <= ack_spi_sync1;

        spi_cs_n_sync_prev <= spi_cs_n_sync;
        fifo_rd_en_spi <= 1'b0; // default: 1-cycle pulse

        if (!spi_cs_n_sync_prev && spi_cs_n_sync) begin
            if (write_pending && !req_valid_spi) req_valid_spi <= 1'b1;
            write_pending <= 1'b0;
        end

        if (spi_cs_n_sync) begin
            bit_counter <= 0;
            spi_shift_reg <= 0;
            spi_state <= SPI_IDLE;
        end else begin
            spi_shift_reg <= {spi_shift_reg[6:0], spi_mosi};
            if (bit_counter == 7) bit_counter <= 0;
            else bit_counter <= bit_counter + 1;
            spi_state <= spi_next_state;

            if (bit_counter == 7) begin
                // ---------------------------------------------------------
                // NOTE: state 전이는 "바이트 완료 후 1 edge 뒤"에 일어나므로,
                // spi_state==X 인 동안 실제로 수신 중인 바이트는 X의 "한 단계 전"
                // 바이트이다. 즉 아래 case label은 원래 의도한 state보다
                // 한 단계 앞선(이전) state로 작성해야 한다:
                //   SPI_IDLE      동안 -> CMD byte 수신중   -> read_mode 캡처
                //   SPI_CMD       동안 -> ADDR_HIGH 수신중  -> addr[15:8]
                //   SPI_ADDR_HIGH 동안 -> ADDR_LOW  수신중  -> addr[7:0], fifo_rd_en_spi
                //   SPI_ADDR_LOW  동안 -> DATA0(상위) 수신중-> wdata[31:24]
                //   SPI_DATA_BYTE0동안 -> DATA1     수신중 -> wdata[23:16]
                //   SPI_DATA_BYTE1동안 -> DATA2     수신중 -> wdata[15:8]
                //   SPI_DATA_BYTE2동안 -> DATA3(LSB)수신중 -> wdata[7:0], write_pending
                // ---------------------------------------------------------
                case (spi_state)
                    SPI_IDLE: begin
                        // command byte 완성: MSB(spi_shift_reg[6]) + 현재 mosi = bit0
                        // -> read/write flag는 byte의 최상위 비트
                        spi_read_mode <= spi_shift_reg[6];
                    end
                    SPI_CMD:
                        reg_addr_spi[15:8] <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_ADDR_HIGH: begin
                        reg_addr_spi[7:0] <= {spi_shift_reg[6:0], spi_mosi};
                        if (spi_read_mode) begin
                            // address phase 종료 -> FIFO read 1회 요청
                            fifo_rd_en_spi <= 1'b1;
                        end
                    end
                    SPI_ADDR_LOW: if (!spi_read_mode)
                        reg_wdata_spi[31:24] <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE0: if (!spi_read_mode)
                        reg_wdata_spi[23:16] <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE1: if (!spi_read_mode)
                        reg_wdata_spi[15:8] <= {spi_shift_reg[6:0], spi_mosi};
                    SPI_DATA_BYTE2: if (!spi_read_mode) begin
                        reg_wdata_spi[7:0] <= {spi_shift_reg[6:0], spi_mosi};
                        write_pending <= 1'b1;
                    end
                    default: ;
                endcase
            end
        end

        if (ack_spi_sync2 && req_valid_spi) req_valid_spi <= 1'b0;

        // FIFO read pulse: clk 도메인에서 이미 안정적으로 latch된 값을 가져옴
        if (fifo_rd_en_spi) begin
            fifo_read_buffer <= fifo_read_data_latched;
        end
    end
end

always_comb begin
    spi_next_state = spi_state;
    if (spi_cs_n_sync) spi_next_state = SPI_IDLE;
    else if (bit_counter == 7) begin
        case (spi_state)
            SPI_IDLE:       spi_next_state = SPI_CMD;
            SPI_CMD:        spi_next_state = SPI_ADDR_HIGH;
            SPI_ADDR_HIGH:  spi_next_state = SPI_ADDR_LOW;
            SPI_ADDR_LOW:   spi_next_state = SPI_DATA_BYTE0;
            SPI_DATA_BYTE0: spi_next_state = SPI_DATA_BYTE1;
            SPI_DATA_BYTE1: spi_next_state = SPI_DATA_BYTE2;
            SPI_DATA_BYTE2: spi_next_state = SPI_DATA_BYTE3;
            SPI_DATA_BYTE3: spi_next_state = SPI_IDLE;
            default:        spi_next_state = SPI_IDLE;
        endcase
    end
end

// MISO: read mode일 때, ADDR_LOW/DATA_BYTE0 state 동안 fifo_read_buffer를 MSB부터 shift-out
// (state 정렬 보정: ADDR_LOW 동안 응답 byte0(상위), DATA_BYTE0 동안 응답 byte1(하위)을 출력)
always_comb begin
    if (spi_read_mode) begin
        case (spi_state)
            SPI_ADDR_LOW:   spi_miso = fifo_read_buffer[15 - bit_counter];
            SPI_DATA_BYTE0: spi_miso = fifo_read_buffer[7  - bit_counter];
            default:        spi_miso = 1'b0;
        endcase
    end else begin
        spi_miso = 1'b0;
    end
end

// =============================================================================
// FIFO read enable (clk 도메인)
//   fifo_rd_en_spi는 spi_sclk 도메인에서 최대 1 spi_sclk 주기(=수십~수백 clk)
//   동안 유지되는 level 신호이므로, 그대로 rd_en에 연결하면 FIFO가
//   여러 번(혹은 거의 전부) pop되는 버그가 발생한다.
//   -> 2-flop synchronizer + rising-edge detect로 1-clk pulse로 변환
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_req_sync1 <= 1'b0;
        rd_req_sync2 <= 1'b0;
        rd_req_sync2_d <= 1'b0;
    end else begin
        rd_req_sync1   <= fifo_rd_en_spi;
        rd_req_sync2   <= rd_req_sync1;
        rd_req_sync2_d <= rd_req_sync2;
    end
end

assign fifo_rd_en = rd_req_sync2 && !rd_req_sync2_d; // rising edge -> 1 clk pulse

// fifo_rd_en이 pulse되는 그 클럭에 "pop되는 값"을 clk 도메인에서 즉시 latch
// (spi_sclk 도메인에서 latch하면 1 spi_sclk 주기(100clk) 후라 이미 다음 원소를
//  가리키게 되는 off-by-one이 발생함)
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) fifo_read_data_latched <= 16'h0;
    else if (fifo_rd_en) fifo_read_data_latched <= fifo_rd_data;
end

// =============================================================================
// Request Synchronization (기존 write 경로 동일)
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_valid_clk_sync1 <= 1'b0; req_valid_clk_sync2 <= 1'b0;
        req_valid_clk_prev <= 1'b0; req_rising_edge_clk <= 1'b0;
    end else begin
        req_valid_clk_sync1 <= req_valid_spi;
        req_valid_clk_sync2 <= req_valid_clk_sync1;
        req_rising_edge_clk <= (req_valid_clk_sync2 && !req_valid_clk_prev);
        req_valid_clk_prev <= req_valid_clk_sync2;
    end
end

// =============================================================================
// Register Write Logic
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_addr_cdc <= 16'h0; reg_wdata_cdc <= 32'h0;
        reg_write_pulse <= 1'b0; write_hold_counter <= 4'd0; ack_clk <= 1'b0;
    end else begin
        if (req_rising_edge_clk) begin
            reg_addr_cdc <= reg_addr_spi;
            reg_wdata_cdc <= reg_wdata_spi;
            reg_write_pulse <= 1'b1;
            write_hold_counter <= 4'd1;
        end else if (write_hold_counter > 0) begin
            write_hold_counter <= write_hold_counter - 1;
            reg_write_pulse <= (write_hold_counter > 1) ? 1'b1 : 1'b0;
        end else begin
            reg_write_pulse <= 1'b0;
        end

        if (write_hold_counter == 1) ack_clk <= 1'b1;
        else if (ack_clk) ack_clk <= 1'b0;
    end
end

assign reg_write = reg_write_pulse;

// =============================================================================
// Acknowledge Synchronization
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin ack_clk_sync1 <= 1'b0; ack_clk_sync2 <= 1'b0; end
    else begin ack_clk_sync1 <= ack_clk; ack_clk_sync2 <= ack_clk_sync1; end
end

// =============================================================================
// Configuration Registers (v2: Weighted 기본 모드 + ECG/EEG/EMG weight)
//   CH0-3 = ECG (weight=2), CH4-5 = EEG (weight=1), CH6-7 = EMG (weight=1)
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < 32; i++) config_registers[i] <= 32'h0;
        config_registers[0] <= 32'h0000_0001;  // system enable
        config_registers[1] <= 32'h0000_00FF;  // channel_enable[7:0] = all 8 ch
        config_registers[2] <= 32'h0000_0002;  // arbiter_mode = 2'b10 (Weighted)

        for (int i = 0; i < 8; i++) config_registers[8+i]  <= 32'h0000_0001; // priority (unused in weighted)

        config_registers[16] <= 32'h0000_0002; // CH0 ECG weight=2
        config_registers[17] <= 32'h0000_0002; // CH1 ECG weight=2
        config_registers[18] <= 32'h0000_0002; // CH2 ECG weight=2
        config_registers[19] <= 32'h0000_0002; // CH3 ECG weight=2
        config_registers[20] <= 32'h0000_0001; // CH4 EEG weight=1
        config_registers[21] <= 32'h0000_0001; // CH5 EEG weight=1
        config_registers[22] <= 32'h0000_0001; // CH6 EMG weight=1
        config_registers[23] <= 32'h0000_0001; // CH7 EMG weight=1
    end else if (reg_write) begin
        automatic int reg_index;
        reg_index = reg_addr_cdc[7:2];
        if (reg_index < 32) config_registers[reg_index] <= reg_wdata_cdc;
    end
end

assign channel_enable = config_registers[1][NUM_CHANNELS-1:0];
assign arbiter_mode   = config_registers[2][1:0];

logic [NUM_CHANNELS-1:0] urgent_mask;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) urgent_mask <= '0;
    else if (reg_write && reg_addr_cdc[7:2] == 6'd3)
        urgent_mask <= reg_wdata_cdc[NUM_CHANNELS-1:0];
end
assign channel_urgent = urgent_mask;

always_comb begin
    for (int i = 0; i < NUM_CHANNELS; i++) begin
        channel_priority[i] = config_registers[8+i][3:0];
        channel_weight[i]   = config_registers[16+i][7:0];
    end
    for (int i = 0; i < 8; i++) trigger_config[i] = config_registers[4+i];
end

assign channel_ready = channel_ready_internal;

// =============================================================================
// Global Timestamp Counter
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) global_timestamp <= 0;
    else global_timestamp <= global_timestamp + 1;
end

// =============================================================================
// ADC Control State Machine (변경 없음)
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE; settling_counter <= 0; captured_data <= 0; locked_channel <= 0;
    end else begin
        current_state <= next_state;
        if (current_state == SELECT_CHANNEL) settling_counter <= settling_counter + 1;
        else settling_counter <= 0;
        if (current_state == CAPTURE_DATA) captured_data <= adc_data;
        // 변환을 "시작"하는 이 사이클의 selected_channel을 latch.
        // (이 시점이 곧 adc_channel_sel로 외부에 알려지는 채널이며,
        //  2클럭 뒤 CAPTURE_DATA에서 weight_accumulator 갱신으로
        //  selected_channel이 바뀌어도 영향받지 않음)
        if (current_state == START_CONVERSION) locked_channel <= selected_channel;
    end
end

always_comb begin
    next_state = current_state;
    adc_start_conv = 1'b0;
    channel_accept = 1'b0;
    sample_ready = 1'b0;
    case (current_state)
        IDLE:
            if (channel_valid && !adc_busy_internal && config_registers[0][0])
                next_state = SELECT_CHANNEL;
        SELECT_CHANNEL:
            if (settling_counter >= 8) next_state = START_CONVERSION;
        START_CONVERSION: begin adc_start_conv = 1'b1; next_state = WAIT_CONVERSION; end
        WAIT_CONVERSION:
            if (adc_conv_done) next_state = CAPTURE_DATA;
        CAPTURE_DATA: begin sample_ready = 1'b1; channel_accept = 1'b1; next_state = NEXT_CHANNEL; end
        NEXT_CHANNEL: next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

// =============================================================================
// FIFO Write Logic
// =============================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_wr_en <= 1'b0; fifo_wr_data <= 16'h0;
    end else begin
        if (sample_ready && !fifo_full) begin
            fifo_wr_en <= 1'b1;
            // captured_data는 CAPTURE_DATA 사이클에 갱신되어 "다음 클럭부터" 반영되므로,
            // 이 사이클(CAPTURE_DATA, sample_ready=1)에서 읽으면 1라운드 전 값(stale)이 됨.
            // selected_channel(이번 라운드 값)과 짝을 맞추기 위해 adc_data를 직접 사용.
            fifo_wr_data <= {1'b0, locked_channel, adc_data}; // CHANNEL_WIDTH=3 -> [1+3+12=16]
        end else fifo_wr_en <= 1'b0;
    end
end

// =============================================================================
// Status and Interrupt
// =============================================================================
assign status_leds = {fifo_full, fifo_empty, trigger_detected, warning_flags[4:0]};
assign interrupt = trigger_detected || fifo_full || (|warning_flags);

// =============================================================================
// Module Instantiations
// =============================================================================
configurable_arbiter #(.NUM_CHANNELS(NUM_CHANNELS)) arbiter_inst (
    .clk(clk), .rst_n(rst_n),
    .arbiter_mode(arbiter_mode),
    .channel_priority(channel_priority),
    .channel_weight(channel_weight),
    .channel_enable(channel_enable),
    .channel_ready(channel_ready),
    .channel_urgent(channel_urgent),
    .adc_busy(adc_busy_internal),
    .selected_channel(selected_channel),
    .channel_valid(channel_valid),
    .channel_accept(channel_accept)
);

single_fifo #(
    .DATA_WIDTH(16), .FIFO_DEPTH(FIFO_DEPTH), .ALMOST_FULL_THRESHOLD(12)
) fifo_inst (
    .clk(clk), .rst_n(rst_n),
    .wr_data(fifo_wr_data), .wr_en(fifo_wr_en), .wr_full(fifo_full), .almost_full(fifo_almost_full),
    .rd_data(fifo_rd_data), .rd_en(fifo_rd_en), .rd_empty(fifo_empty),
    .count(fifo_count)
);

derivative_threshold_engine #(.NUM_CHANNELS(NUM_CHANNELS), .ADC_WIDTH(ADC_WIDTH)) trigger_inst (
    .clk(clk), .rst_n(rst_n),
    .data_in(captured_data), .channel_in(selected_channel), .data_valid(sample_ready),
    .config_reg(trigger_config),
    .trigger_out(trigger_detected), .trigger_confidence(trigger_confidence),
    .trigger_metadata(trigger_metadata), .trigger_valid(trigger_valid)
);

performance_monitor #(.NUM_CHANNELS(NUM_CHANNELS)) perf_mon (
    .clk(clk), .rst_n(rst_n),
    .sample_valid(sample_ready), .sample_channel(selected_channel), .sample_timestamp(global_timestamp),
    .fifo_wr_en(fifo_wr_en), .fifo_rd_en(fifo_rd_en), .fifo_count(fifo_count), .fifo_depth(5'd16),
    .fifo_full(fifo_full), .fifo_empty(fifo_empty),
    .trigger_detected(trigger_detected), .trigger_channel(selected_channel), .trigger_confidence(trigger_confidence),
    .adc_conversion_start(adc_start_conv), .adc_conversion_done(adc_conv_done), .adc_channel(selected_channel),
    .throughput_sps(throughput_sps), .avg_latency_ns(avg_latency_ns), .max_latency_ns(max_latency_ns),
    .fifo_utilization_pct(fifo_utilization_pct), .trigger_rate_ppm(trigger_rate_ppm),
    .warning_flags(warning_flags), .debug_counters(debug_counters)
);

assign adc_channel_sel = selected_channel;

endmodule



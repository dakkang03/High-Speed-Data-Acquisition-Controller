// =============================================================================
// mac_array.sv
// HS-DAQ anomaly-score MAC array
//
// Spec:
//   Input feature map bit-width : 12 bit (unsigned) - biosignal sample,
//                                  matches HS-DAQ RTL's ADC_WIDTH exactly
//   Weight bit-width            :  8 bit (signed)   - anomaly pattern coefficient
//   Output bit-width            : 32 bit (signed)   - per-channel anomaly score
//   MAC array size              : NUM_CHANNELS x WINDOW_SIZE (default 8 x 4)
//
// Operation: per-channel independent dot product (NOT full matrix multiply,
// NOT systolic -- no data shared between channels)
//   result[ch] = sum_{k=0}^{WINDOW_SIZE-1} input_fm[ch][k] * weight[ch][k]
//
// Interface:
//   - valid_in  : 1-cycle pulse to start a new accumulation window.
//                 Caller presents full input_fm and weight matrices on this cycle.
//                 Both are latched internally; caller may change them next cycle.
//   - valid_out : 1-cycle pulse WINDOW_SIZE+1 cycles after valid_in, indicating
//                 result[] is stable and final.
//
// Structure:
//   NUM_CHANNELS instances of mac_cell, running fully in parallel.
//   An internal tap sequencer walks tap_idx 0..WINDOW_SIZE-1 over WINDOW_SIZE
//   cycles, feeding each channel's mac_cell one tap per cycle (one
//   multiply-accumulate per active cycle per cell -- same convention as the
//   uploaded MAC.sv, not a combinational adder tree).
//
// Why not systolic:
//   Each channel's dot product is fully independent (no weight reuse across
//   channels, no psum forwarding between channels). Systolic PE-to-PE
//   forwarding only helps when one weight or psum must reach many PEs over
//   time -- that condition does not exist here, so a parallel MAC bank
//   achieves the same result with lower wiring complexity and lower latency.
// =============================================================================


// -----------------------------------------------------------------------------
// mac_cell: single multiply-accumulate unit (one PE)
//   - Accumulates in_data * w_data into acc over WINDOW_SIZE valid_in cycles
//   - clear resets acc to 0 (start of new window)
//   - in_data is unsigned (raw ADC code) -- zero-extended via {1'b0, in_data}
//     to avoid accidental sign-extension when multiplying against signed weight
// -----------------------------------------------------------------------------
module mac_cell #(
    parameter IN_WIDTH  = 12,  // unsigned
    parameter W_WIDTH   = 8,   // signed
    parameter OUT_WIDTH = 32   // signed
)(
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic  [IN_WIDTH-1:0]        in_data,    // unsigned ADC sample
    input  logic signed [W_WIDTH-1:0]   w_data,     // signed weight
    input  logic                        valid_in,   // accumulate this cycle
    input  logic                        clear,      // reset acc (start of window)
    output logic signed [OUT_WIDTH-1:0] acc,
    output logic                        valid_out
);
    logic signed [OUT_WIDTH-1:0] product;
    // Zero-extend unsigned input with a guard bit so it always looks
    // non-negative to the signed multiplier -- no sign-extension hazard.
    assign product = $signed({1'b0, in_data}) * $signed(w_data);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc       <= '0;
            valid_out <= 1'b0;
        end else if (clear) begin
            acc       <= '0;
            valid_out <= 1'b0;
        end else if (valid_in) begin
            acc       <= acc + product;
            valid_out <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end
endmodule


// -----------------------------------------------------------------------------
// mac_array: NUM_CHANNELS parallel mac_cells driven by an internal tap sequencer
// -----------------------------------------------------------------------------
module mac_array #(
    parameter NUM_CHANNELS = 8,
    parameter WINDOW_SIZE  = 4,
    parameter IN_WIDTH     = 12,
    parameter W_WIDTH      = 8,
    parameter OUT_WIDTH    = 32
)(
    input  logic clk,
    input  logic rst_n,

    input  logic        [IN_WIDTH-1:0] input_fm [0:NUM_CHANNELS-1][0:WINDOW_SIZE-1],
    input  logic signed [W_WIDTH-1:0]  weight   [0:NUM_CHANNELS-1][0:WINDOW_SIZE-1],
    input  logic                       valid_in,

    output logic signed [OUT_WIDTH-1:0] result [0:NUM_CHANNELS-1],
    output logic                        valid_out
);

    // -------------------------------------------------------------------------
    // Latch input_fm and weight when valid_in arrives so the tap sequencer can
    // walk through WINDOW_SIZE taps even if the caller changes inputs next cycle.
    // -------------------------------------------------------------------------
    logic [IN_WIDTH-1:0]       input_fm_lat [0:NUM_CHANNELS-1][0:WINDOW_SIZE-1];
    logic signed [W_WIDTH-1:0] weight_lat   [0:NUM_CHANNELS-1][0:WINDOW_SIZE-1];

    // -------------------------------------------------------------------------
    // Tap sequencer: on valid_in pulse, walks tap_idx 0..WINDOW_SIZE-1 over
    // WINDOW_SIZE cycles feeding all channels in lockstep.
    // -------------------------------------------------------------------------
    logic [1:0] tap_idx;
    logic       running;
    logic       clear_acc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap_idx   <= '0;
            running   <= 1'b0;
            clear_acc <= 1'b0;
        end else begin
            clear_acc <= 1'b0;
            if (valid_in && !running) begin
                input_fm_lat <= input_fm;
                weight_lat   <= weight;
                running      <= 1'b1;
                tap_idx      <= '0;
                clear_acc    <= 1'b1;
            end else if (running && !clear_acc) begin
                // clear ???(clk2)??? tap_idx? ??? ??
                // clk3?? tap 0,1,2,3 ??? 4? ??
                if (tap_idx == WINDOW_SIZE-1) begin
                    running <= 1'b0;
                    tap_idx <= '0;
                end else begin
                    tap_idx <= tap_idx + 1'b1;
                end
            end
        end
    end

    logic mac_valid_in;
    assign mac_valid_in = running && !clear_acc;

    // -------------------------------------------------------------------------
    // NUM_CHANNELS parallel mac_cell instances
    // -------------------------------------------------------------------------
    logic [NUM_CHANNELS-1:0] cell_valid_out;

    genvar ch;
    generate
        for (ch = 0; ch < NUM_CHANNELS; ch++) begin : g_ch
            mac_cell #(
                .IN_WIDTH(IN_WIDTH), .W_WIDTH(W_WIDTH), .OUT_WIDTH(OUT_WIDTH)
            ) u_mac (
                .clk      (clk),
                .rst_n    (rst_n),
                .in_data  (input_fm_lat[ch][tap_idx]),
                .w_data   (weight_lat[ch][tap_idx]),
                .valid_in (mac_valid_in),
                .clear    (clear_acc),
                .acc      (result[ch]),
                .valid_out(cell_valid_out[ch])
            );
        end
    endgenerate

    // valid_out: pulse for 1 cycle when running falls back to 0
    // (i.e. the cycle after tap WINDOW_SIZE-1 was consumed).
    logic running_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) running_d <= 1'b0;
        else        running_d <= running;
    end
    assign valid_out = running_d && !running;

endmodule

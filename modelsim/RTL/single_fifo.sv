module single_fifo #(
    parameter DATA_WIDTH = 16,
    parameter L1_DEPTH = 32,
    parameter L2_DEPTH = 128,
    parameter L3_DEPTH = 512,
    parameter FIFO_DEPTH = 672,
    parameter ADDR_WIDTH = $clog2(FIFO_DEPTH)
)(
    input logic clk,
    input logic rst_n,
    
    input logic [1:0] fifo_mode,
    input logic [7:0] watermark_l1,
    input logic [7:0] watermark_l2,
    input logic [7:0] watermark_l3,
    
    input logic [DATA_WIDTH-1:0] wr_data,
    input logic wr_en,
    output logic wr_full,
    output logic [1:0] wr_level,
    
    output logic [DATA_WIDTH-1:0] rd_data,
    input logic rd_en,
    output logic rd_empty,
    output logic [1:0] rd_level,
    
    output logic [31:0] fifo_status,
    output logic [2:0] level_overflow,
    output logic backpressure_active,
    output logic [ADDR_WIDTH:0] count
);

logic [DATA_WIDTH-1:0] mem [FIFO_DEPTH-1:0];
logic [ADDR_WIDTH:0] wr_ptr, rd_ptr;
logic [ADDR_WIDTH:0] fifo_count;

assign fifo_count = wr_ptr - rd_ptr;
assign wr_full = (fifo_count >= FIFO_DEPTH);
assign rd_empty = (fifo_count == 0);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr <= 0;
        rd_ptr <= 0;
    end else begin
        if (wr_en && !wr_full) begin
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
            wr_ptr <= wr_ptr + 1;
        end
        
        if (rd_en && !rd_empty) begin
            rd_ptr <= rd_ptr + 1;
        end
    end
end

assign rd_data = mem[rd_ptr[ADDR_WIDTH-1:0]];

logic [ADDR_WIDTH:0] l1_count, l2_count, l3_count;

always_comb begin
    if (fifo_count <= L1_DEPTH) begin
        l1_count = fifo_count;
        l2_count = 0;
        l3_count = 0;
    end else if (fifo_count <= L1_DEPTH + L2_DEPTH) begin
        l1_count = L1_DEPTH;
        l2_count = fifo_count - L1_DEPTH;
        l3_count = 0;
    end else begin
        l1_count = L1_DEPTH;
        l2_count = L2_DEPTH;
        l3_count = fifo_count - L1_DEPTH - L2_DEPTH;
    end
end

assign wr_level = (fifo_count < L1_DEPTH) ? 2'b00 :
                  (fifo_count < L1_DEPTH + L2_DEPTH) ? 2'b01 : 2'b10;
assign rd_level = 2'b00;
assign backpressure_active = (fifo_count * 100 >= FIFO_DEPTH * 90);
assign level_overflow = {(fifo_count >= FIFO_DEPTH), 
                        (fifo_count >= L1_DEPTH + L2_DEPTH),
                        (fifo_count >= L1_DEPTH)};
assign fifo_status = {8'h00, l3_count[7:0], l2_count[7:0], l1_count[7:0]};
assign count = fifo_count;

endmodule

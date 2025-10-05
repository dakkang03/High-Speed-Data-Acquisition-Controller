`timescale 1ns/1ps

module tb_high_speed_daq_controller;

localparam NUM_CHANNELS = 16;
localparam ADC_WIDTH = 12;
localparam CLK_PERIOD = 10;
localparam SPI_PERIOD = 100;
localparam MAX_SAMPLES = 10000;

logic clk, rst_n;
logic spi_sclk, spi_mosi, spi_miso, spi_cs_n;
logic adc_start_conv;
logic [3:0] adc_channel_sel;
logic adc_conv_done, adc_busy;
logic [ADC_WIDTH-1:0] adc_data;
logic serial_data, serial_clk, serial_valid;
logic interrupt;
logic [7:0] status_leds;

int error_count = 0;
int total_samples = 0;

logic [ADC_WIDTH-1:0] biosignal_data [0:NUM_CHANNELS-1][0:MAX_SAMPLES-1];
int sample_index = 0;
bit data_loaded = 0;

string test_scenario = "normal";
string base_path = "/home/u1425837/Desktop/ECE_5710_6710_F24/modelsim/RTL/test_data";

logic serial_valid_d;

// Force signal control
bit force_arbiter_signals = 0;
logic [15:0] forced_channel_ready = 16'h0000;

high_speed_daq_controller #(
    .NUM_CHANNELS(16),
    .ADC_WIDTH(12),
    .FIFO_DEPTH(672),
    .CHANNEL_WIDTH(4),
    .TIMESTAMP_WIDTH(32)
) dut (
    .clk(clk),
    .rst_n(rst_n),
    .spi_sclk(spi_sclk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_cs_n(spi_cs_n),
    .adc_start_conv(adc_start_conv),
    .adc_channel_sel(adc_channel_sel),
    .adc_conv_done(adc_conv_done),
    .adc_data(adc_data),
    .adc_busy(adc_busy),
    .serial_data(serial_data),
    .serial_clk(serial_clk),
    .serial_valid(serial_valid),
    .interrupt(interrupt),
    .status_leds(status_leds)
);

// Force arbiter signals to create multiple ready conditions
always @(*) begin
    if (force_arbiter_signals) begin
        force dut.arbiter_inst.channel_ready = forced_channel_ready;
        force dut.arbiter_inst.adc_busy = 1'b0;
    end else begin
        release dut.arbiter_inst.channel_ready;
        release dut.arbiter_inst.adc_busy;
    end
end

// Data Loading
initial begin
    int file;
    string line;
    int sample;
    string filename;
    
    $sformat(filename, "%s/%s_all_channels.csv", base_path, test_scenario);
    $display("Loading biosignal data from %s...", filename);
    
    file = $fopen(filename, "r");
    
    if (file) begin
        void'($fgets(line, file));
        sample = 0;
        while (!$feof(file) && sample < MAX_SAMPLES) begin
            if ($fgets(line, file)) begin
                automatic int ch = 0;
                automatic string token = "";
                
                for (int i = 0; i < line.len(); i++) begin
                    if (line[i] == "," || line[i] == "\n" || i == line.len()-1) begin
                        if (line[i] != "," && line[i] != "\n") token = {token, line[i]};
                        if (token.len() > 0 && ch < NUM_CHANNELS) begin
                            $sscanf(token, "%h", biosignal_data[ch][sample]);
                            ch++;
                        end
                        token = "";
                    end else begin
                        token = {token, line[i]};
                    end
                end
                sample++;
            end
        end
        $fclose(file);
        $display("  Loaded %0d samples", sample);
    end else begin
        $display("  Using random data");
        for (int ch = 0; ch < NUM_CHANNELS; ch++) begin
            for (sample = 0; sample < MAX_SAMPLES; sample++) begin
                biosignal_data[ch][sample] = $urandom_range(0, 4095);
            end
        end
    end
    
    data_loaded = 1;
    $display("Data loading complete\n");
end

initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// ADC Model
initial begin
    adc_conv_done = 0;
    adc_busy = 0;
    adc_data = 0;
    wait(rst_n && data_loaded);
    
    forever begin
        @(posedge adc_start_conv);
        adc_busy = 1;
        repeat(20) @(posedge clk);
        
        if (sample_index < MAX_SAMPLES) begin
            adc_data = biosignal_data[adc_channel_sel][sample_index];
        end else begin
            adc_data = $urandom_range(0, (2**ADC_WIDTH)-1);
        end
        
        @(posedge clk);
        adc_conv_done = 1;
        @(posedge clk);
        adc_conv_done = 0;
        adc_busy = 0;
    end
end

// Sample Counter
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        serial_valid_d <= 1'b0;
        total_samples <= 0;
    end else begin
        serial_valid_d <= serial_valid;
        if (serial_valid_d && !serial_valid) begin
            total_samples = total_samples + 1;
        end
    end
end

always @(posedge clk) begin
    if (serial_valid_d && !serial_valid) begin
        if (adc_channel_sel == NUM_CHANNELS-1) begin
            sample_index = sample_index + 1;
            if (sample_index >= MAX_SAMPLES) sample_index = 0;
        end
    end
end

// SPI Tasks
task spi_write_reg(input [15:0] addr, input [31:0] data);
    begin
        spi_cs_n = 0;
        #(SPI_PERIOD);
        spi_send_byte(8'h01);
        spi_send_byte(addr[15:8]);
        spi_send_byte(addr[7:0]);
        spi_send_byte(data[31:24]);
        spi_send_byte(data[23:16]);
        spi_send_byte(data[15:8]);
        spi_send_byte(data[7:0]);
        #(SPI_PERIOD);
        spi_cs_n = 1;
        repeat(10) @(posedge clk);
    end
endtask

task spi_send_byte(input [7:0] data);
    begin
        for (int i = 7; i >= 0; i--) begin
            spi_mosi = data[i];
            #(SPI_PERIOD/2);
            spi_sclk = 1;
            #(SPI_PERIOD/2);
            spi_sclk = 0;
        end
        #(SPI_PERIOD/4);
    end
endtask

task reset_system();
    begin
        rst_n = 0;
        spi_sclk = 0;
        spi_mosi = 0;
        spi_cs_n = 1;
        force_arbiter_signals = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);
        $display("Reset complete\n");
    end
endtask

// Arbiter stimulus task
task test_arbiter_mode(input [1:0] mode, input string mode_name);
    begin
        $display("--- Arbiter Test: %s ---", mode_name);
        
        // Configure mode
        spi_write_reg(16'h0008, {30'h0, mode});
        repeat(20) @(posedge clk);
        
        // Force multiple channels ready to trigger comparison logic
        force_arbiter_signals = 1;
        
        // Test Case 1: 4 channels ready
        forced_channel_ready = 16'h000F; // Ch 0-3 ready
        repeat(50) @(posedge clk);
        $display("  4ch ready: selected=%0d, valid=%0b", 
                 dut.arbiter_inst.selected_channel, dut.arbiter_inst.channel_valid);
        
        // Test Case 2: 8 channels ready
        forced_channel_ready = 16'h00FF; // Ch 0-7 ready
        repeat(50) @(posedge clk);
        $display("  8ch ready: selected=%0d, valid=%0b", 
                 dut.arbiter_inst.selected_channel, dut.arbiter_inst.channel_valid);
        
        // Test Case 3: All channels ready
        forced_channel_ready = 16'hFFFF;
        repeat(50) @(posedge clk);
        $display("  16ch ready: selected=%0d, valid=%0b", 
                 dut.arbiter_inst.selected_channel, dut.arbiter_inst.channel_valid);
        
        // Test Case 4: Sparse channels
        forced_channel_ready = 16'h5555; // Alternating
        repeat(50) @(posedge clk);
        $display("  Sparse ready: selected=%0d, valid=%0b\n", 
                 dut.arbiter_inst.selected_channel, dut.arbiter_inst.channel_valid);
        
        force_arbiter_signals = 0;
        repeat(20) @(posedge clk);
    end
endtask

// Main Test
initial begin
    $display("\n=== FIXED Coverage Test with Direct Stimulus ===\n");
    
    wait(data_loaded);
    reset_system();
    
    // =========================================================================
    // PART A: DIRECT ARBITER STIMULATION (for coverage)
    // =========================================================================
    
    $display("\n========== PART A: ARBITER COVERAGE TESTS ==========\n");
    
    // Test 1: Mode 0 - Round Robin
    begin : mode0_direct
        spi_write_reg(16'h0004, 32'h0000_FFFF); // Enable all channels
        test_arbiter_mode(2'b00, "Mode 0 Round-Robin");
    end
    
    // Test 2: Mode 1 - Priority with Equal Priorities
    begin : mode1_equal_priority
        spi_write_reg(16'h0004, 32'h0000_FFFF);
        
        // Set equal priorities for some channels
        for (int i = 0; i < 4; i++) begin
            spi_write_reg(16'h0020 + (i*4), 32'h0000_0005); // Priority 5
        end
        for (int i = 4; i < 8; i++) begin
            spi_write_reg(16'h0020 + (i*4), 32'h0000_0005); // Priority 5
        end
        
        test_arbiter_mode(2'b01, "Mode 1 Priority Equal");
    end
    
    // Test 3: Mode 1 - Different Priorities
    begin : mode1_diff_priority
        // Ascending priorities
        for (int i = 0; i < 8; i++) begin
            spi_write_reg(16'h0020 + (i*4), 32'h0000_0000 + i);
        end
        
        test_arbiter_mode(2'b01, "Mode 1 Priority Different");
    end
    
    // Test 4: Mode 1 - Descending Priorities
    begin : mode1_desc_priority
        for (int i = 0; i < 8; i++) begin
            spi_write_reg(16'h0020 + (i*4), 32'h0000_000F - i);
        end
        
        test_arbiter_mode(2'b01, "Mode 1 Priority Descending");
    end
    
    // Test 5: Mode 2 - Weighted Equal
    begin : mode2_equal_weight
        for (int i = 0; i < 8; i++) begin
            spi_write_reg(16'h0040 + (i*4), 32'h0000_0010); // Weight 16
        end
        
        test_arbiter_mode(2'b10, "Mode 2 Weighted Equal");
    end
    
    // Test 6: Mode 2 - Weighted Different
    begin : mode2_diff_weight
        spi_write_reg(16'h0040, 32'h0000_0001); // Ch0: 1
        spi_write_reg(16'h0044, 32'h0000_0080); // Ch1: 128
        spi_write_reg(16'h0048, 32'h0000_0040); // Ch2: 64
        spi_write_reg(16'h004C, 32'h0000_0020); // Ch3: 32
        
        test_arbiter_mode(2'b10, "Mode 2 Weighted Different");
    end
    
    // Test 7: Mode 2 - Zero Weights
    begin : mode2_zero_weight
        spi_write_reg(16'h0040, 32'h0000_0000); // Ch0: 0
        spi_write_reg(16'h0044, 32'h0000_0001); // Ch1: 1
        spi_write_reg(16'h0048, 32'h0000_0000); // Ch2: 0
        spi_write_reg(16'h004C, 32'h0000_0001); // Ch3: 1
        
        test_arbiter_mode(2'b10, "Mode 2 Zero Weight");
    end
    
    // Test 8: Mode 3 - Dynamic with Urgent
    begin : mode3_urgent
        spi_write_reg(16'h000C, 32'h0000_0006); // Ch1, Ch2 urgent
        
        test_arbiter_mode(2'b11, "Mode 3 Dynamic Urgent");
    end
    
    // Test 9: Mode 3 - Dynamic No Urgent (fallback to weighted)
    begin : mode3_no_urgent
        spi_write_reg(16'h000C, 32'h0000_0000); // No urgent
        spi_write_reg(16'h0040, 32'h0000_0010);
        spi_write_reg(16'h0044, 32'h0000_0020);
        spi_write_reg(16'h0048, 32'h0000_0030);
        
        test_arbiter_mode(2'b11, "Mode 3 Dynamic Fallback");
    end
    
    // Test 10: Mode 3 - Multiple Urgent
    begin : mode3_multi_urgent
        spi_write_reg(16'h000C, 32'h0000_00FF); // All urgent
        
        test_arbiter_mode(2'b11, "Mode 3 Multi Urgent");
    end
    
    // Test 11: Channel Enable Runtime Change
    begin : runtime_enable_change
        $display("--- Test: Runtime Enable Change ---");
        
        force_arbiter_signals = 1;
        spi_write_reg(16'h0008, 32'h0000_0000); // Mode 0
        
        // Change enable mask during operation
        spi_write_reg(16'h0004, 32'h0000_000F); // Ch 0-3
        forced_channel_ready = 16'h000F;
        repeat(50) @(posedge clk);
        
        spi_write_reg(16'h0004, 32'h0000_00F0); // Ch 4-7
        forced_channel_ready = 16'h00F0;
        repeat(50) @(posedge clk);
        
        spi_write_reg(16'h0004, 32'h0000_FFFF); // All
        forced_channel_ready = 16'hFFFF;
        repeat(50) @(posedge clk);
        
        $display("  Runtime enable change complete\n");
        force_arbiter_signals = 0;
    end
    
    // =========================================================================
    // PART B: NORMAL OPERATION TESTS (functional verification)
    // =========================================================================
    
    $display("\n========== PART B: FUNCTIONAL TESTS ==========\n");
    
    // Test 12: Normal Mode 0 Operation
    begin : normal_mode0
        $display("--- Test: Normal Mode 0 Operation ---");
        spi_write_reg(16'h0008, 32'h0000_0000);
        spi_write_reg(16'h0000, 32'h0000_0001); // Enable system
        spi_write_reg(16'h0004, 32'h0000_000F); // Ch 0-3
        
        repeat(5000) @(posedge clk);
        $display("  Samples collected: %0d\n", total_samples);
        
        spi_write_reg(16'h0000, 32'h0000_0000); // Stop
        repeat(1000) @(posedge clk);
    end
    
    // Test 13: FIFO Fill Test
    begin : fifo_fill_test
        $display("--- Test: FIFO Fill Levels ---");
        
        spi_write_reg(16'h0000, 32'h0000_0001);
        spi_write_reg(16'h0004, 32'h0000_FFFF); // All channels
        
        // Monitor FIFO levels
        fork
            begin
                wait(dut.fifo_inst.fifo_count >= 32);
                $display("  FIFO L1 threshold reached: %0d", dut.fifo_inst.fifo_count);
            end
            begin
                wait(dut.fifo_inst.fifo_count >= 160);
                $display("  FIFO L2 threshold reached: %0d", dut.fifo_inst.fifo_count);
            end
            begin
                wait(dut.fifo_inst.fifo_count >= 400);
                $display("  FIFO L3 threshold reached: %0d", dut.fifo_inst.fifo_count);
            end
            begin
                repeat(10000) @(posedge clk);
            end
        join_any
        disable fork;
        
        $display("  Final FIFO count: %0d\n", dut.fifo_inst.fifo_count);
        
        spi_write_reg(16'h0000, 32'h0000_0000);
        repeat(3000) @(posedge clk);
    end
    
    repeat(100) @(posedge clk);
    
    // Summary
    $display("\n=== FIXED Coverage Test Summary ===");
    $display("Total Samples: %0d", total_samples);
    $display("Errors: %0d", error_count);
    $display("\n*** COVERAGE TEST COMPLETE ***\n");
    
    $finish;
end

initial begin
    #20000000; // 20ms timeout
    $display("\n!!! TIMEOUT !!!");
    $display("Total samples: %0d", total_samples);
    $finish;
end

endmodule

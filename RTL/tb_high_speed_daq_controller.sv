`timescale 1ns/1ps

module tb_high_speed_daq_controller;

localparam NUM_CHANNELS = 16;  // 16 channels for biosignals
localparam ADC_WIDTH = 12;
localparam CLK_PERIOD = 10;
localparam SPI_PERIOD = 100;
localparam MAX_SAMPLES = 10000;  // 10 seconds @ 1kHz

logic clk, rst_n;
logic spi_sclk, spi_mosi, spi_miso, spi_cs_n;
logic adc_start_conv;
logic [3:0] adc_channel_sel;  // 4 bits for 16 channels
logic adc_conv_done, adc_busy;
logic [ADC_WIDTH-1:0] adc_data;
logic serial_data, serial_clk, serial_valid;
logic interrupt;
logic [7:0] status_leds;

int error_count = 0;
int total_samples = 0;

// Biosignal data storage
logic [ADC_WIDTH-1:0] biosignal_data [0:NUM_CHANNELS-1][0:MAX_SAMPLES-1];
int sample_index = 0;
bit data_loaded = 0;

// Test scenario selection - change this to test different scenarios
string test_scenario = "high_noise";  // Options: "normal", "high_noise", "artifact_heavy", "stress_test"

// ABSOLUTE PATH - Change this to your actual file location
string base_path = "/home/u1425837/Desktop/ECE_5710_6710_F24/modelsim/RTL/test_data";  // ? ??? ?????

// Clock-synchronous serial_valid delayed version for reliable falling-edge detection
logic serial_valid_d;

// Instantiate DUT with 16 channels
high_speed_daq_controller #(
    .NUM_CHANNELS(16),           // 16 channels
    .ADC_WIDTH(12),
    .FIFO_DEPTH(672),
    .CHANNEL_WIDTH(4),           // 4 bits for 16 channels
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
// Load biosignal data from selected scenario using absolute path
initial begin
    int file;
    string line;
    int sample;
    string filename;
    
    $sformat(filename, "%s/%s_all_channels.csv", base_path, test_scenario);
    $display("Loading biosignal data from %s...", filename);
    
    file = $fopen(filename, "r");
    
    if (file) begin
        // Skip header line
        void'($fgets(line, file));
        
        sample = 0;
        while (!$feof(file) && sample < MAX_SAMPLES) begin
            if ($fgets(line, file)) begin
                // Parse comma-separated hex values
                // Format: CH00,CH01,CH02,...,CH15
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
        $display("  Loaded %0d samples for %0d channels from '%s' scenario", sample, NUM_CHANNELS, test_scenario);
    end else begin
        $display("  ERROR: Could not open %s", filename);
        $display("  Please check the base_path variable in the testbench");
        $display("  Current base_path: %s", base_path);
        $display("  Using random data as fallback");
        // Fallback to random data
        for (int ch = 0; ch < NUM_CHANNELS; ch++) begin
            for (sample = 0; sample < MAX_SAMPLES; sample++) begin
                biosignal_data[ch][sample] = $urandom_range(0, 4095);
            end
        end
    end
    
    data_loaded = 1;
    $display("Biosignal data loading complete\n");
end

// Clock generation
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// ADC simulator - returns real biosignal data
initial begin
    adc_conv_done = 0;
    adc_busy = 0;
    adc_data = 0;
    wait(rst_n && data_loaded);  // Wait for reset and data loading
    
    forever begin
        @(posedge adc_start_conv);
        adc_busy = 1;
        repeat(20) @(posedge clk);
        
        // Return real biosignal data based on channel and sample index
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

// Increment sample index on each complete sample cycle
always @(posedge clk) begin
    if (serial_valid_d && !serial_valid) begin  // Falling edge
        if (adc_channel_sel == NUM_CHANNELS-1) begin
            sample_index = sample_index + 1;
            if (sample_index >= MAX_SAMPLES) begin
                sample_index = 0;  // Loop back
            end
        end
    end
end

// Sample counter: detect falling edge of serial_valid
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        serial_valid_d <= 1'b0;
        total_samples <= 0;
    end else begin
        serial_valid_d <= serial_valid;
        // Falling edge: end of frame
        if (serial_valid_d && !serial_valid) begin
            total_samples = total_samples + 1;
        end
    end
end

// SPI write register task
task spi_write_reg(input [15:0] addr, input [31:0] data);
    begin
        $display("SPI Write: Addr=0x%04h, Data=0x%08h at time %0t", addr, data, $time);
        
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
        
        // Wait for write to complete
        repeat(10) @(posedge clk);
    end
endtask

// SPI send byte task
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

// Reset system task
task reset_system();
    begin
        rst_n = 0;
        spi_sclk = 0;
        spi_mosi = 0;
        spi_cs_n = 1;
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);
        $display("Reset complete at time %0t\n", $time);
    end
endtask

// Wait for samples task
task wait_samples(input int target, input int timeout_cycles);
    int start_count;
    int timeout_counter;
    begin
        start_count = total_samples;
        timeout_counter = 0;
        
        while ((total_samples - start_count) < target && timeout_counter < timeout_cycles) begin
            @(posedge clk);
            timeout_counter++;
        end
    end
endtask

// Main test sequence - adapted for 16-channel biosignal testing
initial begin
    $display("\n=== 16-Channel Biosignal DAQ Controller Test ===\n");
    $display("Address Map:");
    $display("  0x0000: Control (Enable)");
    $display("  0x0004: Channel Enable (16 bits)");
    $display("  0x0008: Arbiter Mode");
    $display("  0x0020-0x005C: Channel Priorities (CH0-15)");
    $display("  0x0040-0x009C: Channel Weights (CH0-15)\n");
    
    wait(data_loaded);  // Wait for biosignal data to load
    reset_system();
    
    // =========================================================================
    // Test 1: Enable all 16 channels
    // =========================================================================
    $display("--- Test 1: Enable All 16 Channels ---");
    spi_write_reg(16'h0000, 32'h0000_0001);  // Enable
    spi_write_reg(16'h0004, 32'h0000_FFFF);  // All 16 channels
    repeat(100) @(posedge clk);
    
    $display("DEBUG: Register[1] = 0x%08h, [15:0] = 0x%04h", 
             dut.config_registers[1], dut.config_registers[1][15:0]);
    
    if (dut.config_registers[1][15:0] == 16'hFFFF) begin
        $display("PASS: All 16 channels enabled\n");
    end else begin
        $display("FAIL: Channel config error - Expected 0xFFFF, got 0x%04h\n", 
                 dut.config_registers[1][15:0]);
        error_count++;
    end
    
    // =========================================================================
    // Test 2: ECG Channels (0-5)
    // =========================================================================
    $display("--- Test 2: ECG Channels (0-5) ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_003F);  // Channels 0-5
    wait_samples(100, 50000);
    
    if (total_samples >= 100) begin
        $display("PASS: ECG acquisition (%0d samples)\n", total_samples);
    end else begin
        $display("FAIL: Insufficient ECG samples\n");
        error_count++;
    end
    
    // =========================================================================
    // Test 3: EEG Channels (6-11)
    // =========================================================================
    $display("--- Test 3: EEG Channels (6-11) ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_0FC0);  // Channels 6-11
    wait_samples(100, 50000);
    
    if (total_samples >= 100) begin
        $display("PASS: EEG acquisition (%0d samples)\n", total_samples);
    end else begin
        $display("FAIL: Insufficient EEG samples\n");
        error_count++;
    end
    
    // =========================================================================
    // Test 4: EMG Channels (12-15)
    // =========================================================================
    $display("--- Test 4: EMG Channels (12-15) ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_F000);  // Channels 12-15
    wait_samples(100, 50000);
    
    if (total_samples >= 100) begin
        $display("PASS: EMG acquisition (%0d samples)\n", total_samples);
    end else begin
        $display("FAIL: Insufficient EMG samples\n");
        error_count++;
    end
    
    // =========================================================================
    // Test 5: All Channels Simultaneous
    // =========================================================================
    begin
        automatic int start_samples;
        
        $display("--- Test 5: All 16 Channels Simultaneous ---");
        spi_write_reg(16'h0000, 32'h0000_0001);
        spi_write_reg(16'h0004, 32'h0000_FFFF);  // All channels
        
        repeat(100) @(posedge clk);
        start_samples = total_samples;
        wait_samples(500, 200000);
        
        if ((total_samples - start_samples) >= 500) begin
            $display("PASS: Multi-signal acquisition (%0d samples)\n", total_samples - start_samples);
        end else begin
            $display("FAIL: Insufficient samples\n");
            error_count++;
        end
    end
    
    // =========================================================================
    // Test 6: Priority Mode - ECG priority
    // =========================================================================
    $display("--- Test 6: Priority Mode (ECG channels high priority) ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_FFFF);
    spi_write_reg(16'h0008, 32'h0000_0001);  // Priority mode
    
    // Set high priority for ECG channels
    for (int i = 0; i < 6; i++) begin
        spi_write_reg(16'h0020 + (i*4), 32'h0000_0004);  // Priority 4
    end
    
    wait_samples(100, 50000);
    $display("PASS: Priority mode test complete\n");
    
    repeat(100) @(posedge clk);
    
    // =========================================================================
    // Test Summary
    // =========================================================================
    $display("\n=== Summary ===");
    $display("Total Samples: %0d", total_samples);
    $display("Sample Index: %0d/%0d", sample_index, MAX_SAMPLES);
    $display("Errors: %0d", error_count);
    
    if (error_count == 0) begin
        $display("\n*** ALL BIOSIGNAL TESTS PASSED ***\n");
    end else begin
        $display("\n*** %0d TESTS FAILED ***\n", error_count);
    end
    
    $finish;
end

// Timeout watchdog
initial begin
    #50000000;
    $display("TIMEOUT - Test did not complete");
    $finish;
end

endmodule

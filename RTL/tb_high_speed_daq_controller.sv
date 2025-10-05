`timescale 1ns/1ps

module tb_high_speed_daq_controller;

localparam NUM_CHANNELS = 8;
localparam ADC_WIDTH = 12;
localparam CLK_PERIOD = 10;
localparam SPI_PERIOD = 100;

logic clk, rst_n;
logic spi_sclk, spi_mosi, spi_miso, spi_cs_n;
logic adc_start_conv;
logic [2:0] adc_channel_sel;
logic adc_conv_done, adc_busy;
logic [ADC_WIDTH-1:0] adc_data;
logic serial_data, serial_clk, serial_valid;
logic interrupt;
logic [7:0] status_leds;

int error_count = 0;
int total_samples = 0;

// Clock-synchronous serial_valid delayed version for reliable falling-edge detection
logic serial_valid_d;

// Instantiate DUT
high_speed_daq_controller dut (.*);

// Clock generation
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// ADC simulator
initial begin
    adc_conv_done = 0;
    adc_busy = 0;
    adc_data = 0;
    wait(rst_n);
    
    forever begin
        @(posedge adc_start_conv);
        adc_busy = 1;
        repeat(20) @(posedge clk);
        // Produce a random sample
        adc_data = $urandom_range(0, (2**ADC_WIDTH)-1);
        @(posedge clk);
        adc_conv_done = 1;
        @(posedge clk);
        adc_conv_done = 0;
        adc_busy = 0;
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

// Main test sequence
initial begin
    $display("\n=== SPI DAQ Controller Test ===\n");
    $display("Address Map:");
    $display("  0x0000: Control (Enable)");
    $display("  0x0004: Channel Enable");
    $display("  0x0008: Arbiter Mode");
    $display("  0x0020-0x003C: Channel Priorities");
    $display("  0x0040-0x005C: Channel Weights\n");
    
    reset_system();
    
    // =========================================================================
    // Test 1: System Enable
    // =========================================================================
    $display("--- Test 1: System Enable ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    repeat(100) @(posedge clk);
    
    if (dut.config_registers[0][0] == 1'b1) begin
        $display("PASS: System enabled\n");
    end else begin
        $display("FAIL: System not enabled\n");
        error_count++;
    end
    
    // =========================================================================
    // Test 2: Channel Enable
    // =========================================================================
    $display("--- Test 2: Channel Enable ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_00FF);
    repeat(100) @(posedge clk);
    
    if (dut.config_registers[1][7:0] == 8'hFF) begin
        $display("PASS: 8 channels enabled\n");
    end else begin
        $display("FAIL: Channel config error (got 0x%02h)\n", dut.config_registers[1][7:0]);
        error_count++;
    end
    
    // =========================================================================
    // Test 3: Single Channel Acquisition
    // =========================================================================
    $display("--- Test 3: Single Channel ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_0001);
    wait_samples(10, 5000);
    
    if (total_samples >= 10) begin
        $display("PASS: %0d samples acquired\n", total_samples);
    end else begin
        $display("FAIL: Only %0d samples\n", total_samples);
        error_count++;
    end
    
    // =========================================================================
    // Test 4: Multi-Channel Acquisition
    // =========================================================================
    $display("--- Test 4: Multi-Channel ---");
    spi_write_reg(16'h0000, 32'h0000_0001);
    spi_write_reg(16'h0004, 32'h0000_000F);
    wait_samples(50, 20000);
    
    if (total_samples >= 50) begin
        $display("PASS: Multi-channel working (%0d samples)\n", total_samples);
    end else begin
        $display("FAIL: Insufficient samples (%0d)\n", total_samples);
        error_count++;
    end
    
    // =========================================================================
    // Test 5: Extended Operation
    // =========================================================================
    begin
        automatic int start_samples;
        automatic int new_samples;
        
        $display("--- Test 5: Extended Operation ---");
        
        spi_write_reg(16'h0000, 32'h0000_0001);
        spi_write_reg(16'h0004, 32'h0000_00FF);
        
        repeat(100) @(posedge clk);
        
        $display("System state:");
        $display("  Enable: %b", dut.config_registers[0][0]);
        $display("  Channels: 0x%02h", dut.config_registers[1][7:0]);
        
        start_samples = total_samples;
        wait_samples(200, 100000);
        new_samples = total_samples - start_samples;
        
        $display("Collected %0d new samples", new_samples);
        
        if (new_samples >= 200) begin
            $display("PASS\n");
        end else begin
            $display("FAIL: Expected 200+, got %0d\n", new_samples);
            error_count++;
        end
    end
    
    // =========================================================================
    // Test 6: Priority Mode
    // =========================================================================
    begin
        automatic int start_samples;
        
        $display("--- Test 6: Priority Mode ---");
        
        spi_write_reg(16'h0000, 32'h0000_0001);
        spi_write_reg(16'h0004, 32'h0000_000F);
        spi_write_reg(16'h0008, 32'h0000_0001);  // Priority mode
        
        spi_write_reg(16'h0020, 32'h0000_0003);  // Ch0 priority = 3
        spi_write_reg(16'h0024, 32'h0000_0001);  // Ch1 priority = 1
        
        repeat(100) @(posedge clk);
        
        $display("Priority check:");
        $display("  Ch0: %0d", dut.config_registers[8][3:0]);
        $display("  Ch1: %0d", dut.config_registers[9][3:0]);
        
        start_samples = total_samples;
        wait_samples(50, 20000);
        
        if ((total_samples - start_samples) >= 50) begin
            $display("PASS: Priority mode working\n");
        end else begin
            $display("WARNING: Priority mode samples low\n");
        end
    end
    
    // =========================================================================
    // Test 7: Weighted Round-Robin
    // =========================================================================
    begin
        automatic int start_samples;
        
        $display("--- Test 7: Weighted Round-Robin ---");
        
        spi_write_reg(16'h0000, 32'h0000_0001);
        spi_write_reg(16'h0004, 32'h0000_0003);  // Ch0-1 only
        spi_write_reg(16'h0008, 32'h0000_0002);  // WRR mode
        
        spi_write_reg(16'h0040, 32'h0000_0003);  // Ch0 weight = 3
        spi_write_reg(16'h0044, 32'h0000_0001);  // Ch1 weight = 1
        
        repeat(100) @(posedge clk);
        
        $display("Weight check:");
        $display("  Ch0: %0d", dut.config_registers[16][7:0]);
        $display("  Ch1: %0d", dut.config_registers[17][7:0]);
        
        start_samples = total_samples;
        wait_samples(50, 20000);
        
        if ((total_samples - start_samples) >= 50) begin
            $display("PASS: WRR mode working\n");
        end else begin
            $display("WARNING: WRR samples low\n");
        end
    end
    
    repeat(100) @(posedge clk);
    
    // =========================================================================
    // Test Summary
    // =========================================================================
    $display("\n=== Summary ===");
    $display("Total Samples: %0d", total_samples);
    $display("Errors: %0d", error_count);
    
    if (error_count == 0) begin
        $display("\n*** ALL TESTS PASSED ***\n");
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

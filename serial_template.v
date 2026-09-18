module serial_template (
    input sys_clk,            // clk input  (27 MHz on the Tang Nano 9K)
    input sys_rst_n,          // reset input (active low)
    output [5:0] bled,        // 6 board LEDs pin
    input [3:0] sw,
    input [3:0] btn,
    output [7:0] led,         // 8 mother-board LEDs : show ASCII of last key
    output reg [7:0] seven,
    output reg [3:0] segment,
    input uart_rx
);

parameter CLK_FRE  = 27;                 // MHz
parameter UART_FRE = 115200;             // bits/s
// One second expressed in system-clock cycles. 27 MHz -> 27,000,000.
// Exposed as a parameter only so it can be shrunk in simulation; the
// board build uses the default and counts real seconds.
parameter ONE_SEC  = CLK_FRE * 1000000;

reg [6:0]  hex2seven[15:0];
reg [15:0] dispCounter;
reg [1:0]  segCounter;

// ---------------------------------------------------------------------------
// Timer value, held as four BCD digits and shown as  MM:SS
//      d3 d2 : d1 d0
//      |  |     |  +-- seconds units
//      |  |     +----- seconds tens
//      |  +----------- minutes units  (the colon/dot sits after this digit)
//      +-------------- minutes tens
// ---------------------------------------------------------------------------
reg [3:0] d3, d2, d1, d0;

wire [3:0] DISP3;
wire [3:0] DISP2;
wire [3:0] DISP1;
wire [3:0] DISP0;

assign DISP3 = d3;
assign DISP2 = d2;
assign DISP1 = d1;
assign DISP0 = d0;

assign bled = 6'b111111;

// ---------------------------------------------------------------------------
// UART receiver. rx_data carries the ASCII byte of the last key pressed,
// rx_data_valid pulses high for one clock when a new byte has arrived.
// The 8 mother-board LEDs always mirror that byte (so '9' -> 0011 1001).
// ---------------------------------------------------------------------------
wire [7:0] rx_data;
wire       rx_data_valid;
wire       rx_data_ready;

assign rx_data_ready = 1'b1;
assign led           = rx_data;

uart_rx#
(
	.CLK_FRE(CLK_FRE),
	.BAUD_RATE(UART_FRE)
) uart_rx_inst
(
	.clk                        (sys_clk                  ),
	.rst_n                      (sys_rst_n                ),
	.rx_data                    (rx_data                  ),
	.rx_data_valid              (rx_data_valid            ),
	.rx_data_ready              (rx_data_ready            ),
	.rx_pin                     (uart_rx                  )
);

// A received byte is a decimal digit key when it is in the range '0'..'9'
// (ASCII 0x30..0x39). The low nibble then holds the digit value 0..9.
wire        digit_key = (rx_data >= 8'h30) && (rx_data <= 8'h39);
wire [3:0]  key_digit = rx_data[3:0];

// ---------------------------------------------------------------------------
// BUTTON0 press detection. The button is synchronised to sys_clk and any
// clean transition is treated as a press request. Using "either edge" plus
// the guards below makes the start work no matter how the button is wired
// (active-high or active-low) and is immune to a held button / contact bounce.
// ---------------------------------------------------------------------------
reg btn0_s0, btn0_s1, btn0_s2;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        btn0_s0 <= 1'b0;
        btn0_s1 <= 1'b0;
        btn0_s2 <= 1'b0;
    end else begin
        btn0_s0 <= btn[0];
        btn0_s1 <= btn0_s0;
        btn0_s2 <= btn0_s1;
    end
end
wire btn0_edge = (btn0_s1 != btn0_s2);

// ---------------------------------------------------------------------------
// Running flag and the one-second countdown tick. The tick counter only runs
// while the timer is running and is held cleared otherwise, so the very first
// second after BUTTON0 is a full second.
// ---------------------------------------------------------------------------
reg         running;
reg [24:0]  tick_cnt;
wire        is_zero = (d3 == 4'd0) && (d2 == 4'd0) && (d1 == 4'd0) && (d0 == 4'd0);

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        tick_cnt <= 25'd0;
    else if (!running)
        tick_cnt <= 25'd0;
    else if (tick_cnt == ONE_SEC - 1)
        tick_cnt <= 25'd0;
    else
        tick_cnt <= tick_cnt + 25'd1;
end

wire tick = running && (tick_cnt == ONE_SEC - 1);

// ---------------------------------------------------------------------------
// Main control
//   * while stopped : digit keys shift the start time in from the right
//   * BUTTON0       : arm the countdown if a non-zero value has been set
//   * while running : decrement once per second and stop exactly at 00:00
// ---------------------------------------------------------------------------
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        d3      <= 4'd0;
        d2      <= 4'd0;
        d1      <= 4'd0;
        d0      <= 4'd0;
        running <= 1'b0;
    end else begin
        // Start the countdown on a BUTTON0 press (ignored if value is 00:00).
        if (btn0_edge && !running && !is_zero)
            running <= 1'b1;

        if (!running) begin
            // ---- SET mode : shift one new digit in at the right --------------
            if (digit_key && rx_data_valid) begin
                d3 <= d2;
                d2 <= d1;
                d1 <= d0;
                d0 <= key_digit;
            end
        end else begin
            // ---- RUN mode : one BCD decrement per second --------------------
            if (tick) begin
                if (is_zero) begin
                    running <= 1'b0;             // safety, should not be reached
                end else begin
                    if (d0 != 4'd0) begin
                        d0 <= d0 - 4'd1;
                    end else begin
                        d0 <= 4'd9;
                        if (d1 != 4'd0) begin
                            d1 <= d1 - 4'd1;
                        end else begin
                            d1 <= 4'd5;
                            if (d2 != 4'd0) begin
                                d2 <= d2 - 4'd1;
                            end else begin
                                d2 <= 4'd9;
                                if (d3 != 4'd0)
                                    d3 <= d3 - 4'd1;
                            end
                        end
                    end
                    // 00:01 --> 00:00 : reached zero, stop the countdown.
                    if (d3 == 4'd0 && d2 == 4'd0 && d1 == 4'd0 && d0 == 4'd1)
                        running <= 1'b0;
                end
            end
        end
    end
end

// ===========================================================================
// Seven-segment ROM, refresh divider and digit multiplexer (from template).
// The decimal point of DISP2 is lit to act as the MM:SS colon.
// ===========================================================================
initial begin
	hex2seven[0]  = 7'b00111111;
	hex2seven[1]  = 7'b00000110;
	hex2seven[2]  = 7'b01011011;
	hex2seven[3]  = 7'b01001111;
	hex2seven[4]  = 7'b01100110;
	hex2seven[5]  = 7'b01101101;
	hex2seven[6]  = 7'b01111101;
	hex2seven[7]  = 7'b00000111;
	hex2seven[8]  = 7'b01111111;
	hex2seven[9]  = 7'b01101111;
	hex2seven[10] = 7'b01110111;
	hex2seven[11] = 7'b01111100;
	hex2seven[12] = 7'b00111001;
	hex2seven[13] = 7'b01011110;
	hex2seven[14] = 7'b01111001;
	hex2seven[15] = 7'b01110001;
	segment = 4'b0001;
	d3 = 4'd0; d2 = 4'd0; d1 = 4'd0; d0 = 4'd0;
	running = 1'b0;
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        dispCounter <= 16'd0;
        segCounter <= 2'd0;
    end
    else begin
        dispCounter <= dispCounter + 16'd1;
        if (dispCounter == 16'd0) begin
            segCounter <= segCounter + 2'd1;
        end
    end
end

always @(segCounter or DISP0 or DISP1 or DISP2 or DISP3) begin
	case (segCounter)
	2'b00 : begin seven <= {1'b0, hex2seven[DISP0]}; segment <= 4'b0001; end
	2'b01 : begin seven <= {1'b0, hex2seven[DISP1]}; segment <= 4'b0010; end
	2'b10 : begin seven <= {1'b1, hex2seven[DISP2]}; segment <= 4'b0100; end
	2'b11 : begin seven <= {1'b0, hex2seven[DISP3]}; segment <= 4'b1000; end
	endcase
end

endmodule

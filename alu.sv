// alu takes 32-bit inputs a and b and 2-bit control input called ALUControl. The outputs are
// 32-bit Result and 4-bit ALUFlags. The alu has 4 modes which are set by ALUControl:
// 00 -> Result = a + b;
// 01 -> Result = a - b;
// 00 -> Result = a & b;
// 01 -> Result = a | b;
// ALUFlags are then set for each case whereby:
// ALUFlags = {result is negative, result is 0, adder produces carryout, adder results in overflow}

module alu(input logic [31:0] a, b,
		input logic [1:0] ALUControl,
		output logic [31:0] Result,
		output logic [3:0] ALUFlags);

		// define extra logic variables to hold some intermediate values
		// sum: holds 32-bit sum or difference (depending on mode)
		// cout: 1-bit carryout check
		// overflow: 1-bit overflow check
		// bNew: holds 32-bit 1's complement value of b in case of subtraction
		logic [31:0] sum;
		logic cout;
		logic overflow;
		logic [31:0] bNew;

		// always_comb with a case statement that will implement sections of code
		// depending on the value of ALUControl
		// in all cases:
		// the NEGATIVE flag is calculated by checking the Result's last bit
		// the ZERO flag is calculated by checking if every bit of Result is 0
		// the carryout flag is calculated by checking the value of cout
		// the overflow flag is calculated by checking the value of overflow
		// wherever bNew is not used, it is set to 32'b0
		always_comb begin
			case (ALUControl)
				// ADD
				// concatenation of the value of a+b: first 32-bits will go into sum and the
				// remaining bit (if any) will go into cout since that would be the carryout
				// Result of a+b would just be the sum found above
				// overflow is detected through XNOR of the last bit of a and the last bit b and 0
				//	AND XOR of the last bit of a and the last bit of sum
				2'b00: begin //ADD
					{cout, sum} = a + b;
					Result = sum;
					overflow = ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
					ALUFlags[3:0] = {Result[31], (Result == 32'b0), cout, overflow};
					bNew = 32'b0;
				end

				// SUBTRACT
				// bNew: holds 32-bit 1's complement value of b
				// concatenation of the value of a+1+bNew: first 32-bits will go into sum and the
				// remaining bit (if any) will go into cout since that would be the carryout
				// Result of a-b would just be the sum found above
				// overflow is detected through XNOR of the last bit of a and the last bit b and 1
				//	AND XOR of the last bit of a and the last bit of sum
				2'b01: begin // SUBTRACT
					bNew = ~b;
					{cout, sum} = a + 1 + bNew;
					Result = sum;
					overflow = ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
					ALUFlags[3:0] = {Result[31], (Result == 32'b0), cout, overflow};
				end

				// AND
				// Result of a AND b would just be a&b
				// we have no overflow, cout, sum and hence they are set to 0
				2'b10: begin // AND
					Result = a & b;
					overflow = 1'b0;
					cout = 1'b0;
					ALUFlags[3:0] = {Result[31], (Result == 32'b0), cout, overflow};
					sum = 32'b0;
					bNew = 32'b0;
				end

				// OR
				// Result of a OR b would just be a|b
				// we have no overflow, cout, sum and hence they are set to 0
				2'b11: begin // OR
					Result = a | b;
					overflow = 1'b0;
					cout = 1'b0;
					ALUFlags[3:0] = {Result[31], (Result == 32'b0), cout, overflow};
					sum = 32'b0;
					bNew = 32'b0;
				end
				// DEFAULT case sets everything to don't cares
				default: begin
					Result = 32'bx;
					ALUFlags[3:0] = 4'bx;
					sum = 32'bx;
					overflow = 1'bx;
					cout = 1'bx;
					bNew = 32'bx;
				end

		 endcase
	 end // always_comb
endmodule // alu

// alu_testbench test against a set of appropriate test vectors to convince a reasonable person that
// the alu design is correct
// similarly to alu:
// alu_testbench takes 32-bit inputs a and b and 2-bit control input called ALUControl. The outputs are
// 32-bit Result and 4-bit ALUFlags. The alu has 4 modes which are set by ALUControl:
// 00 -> Result = a + b;
// 01 -> Result = a - b;
// 00 -> Result = a & b;
// 01 -> Result = a | b;
// ALUFlags are then set for each case whereby:
// ALUFlags = {result is negative, result is 0, adder produces carryout, adder results in overflow}
module alu_testbench();

		logic [31:0] a, b;
		logic [1:0] ALUControl;
		logic [31:0] Result;
		logic [3:0] ALUFlags;

		logic clk;
		logic [103:0] testvectors [1000:0];

		// alu dut takes the exact same inputs and outputs described above
		alu dut (.*);

		// set up the clock
		parameter CLOCK_PERIOD=100;

		initial clk = 1;
		always begin
		 #(CLOCK_PERIOD/2);
		 clk = ~clk;
		end

		initial begin
			// used to read the aforementioned set of test vectors and stores them in testvectors
			$readmemh("alu.tv", testvectors);

			// i: 0 -> 16, because we have 16 test vectors in alu.tv
			for(int i = 0; i < 17; i = i + 1) begin
				{ALUControl, a, b, Result, ALUFlags} = testvectors[i]; @(posedge clk);
			end
			$stop; // stops the simulation to keep the clock from going to infinity
		end
endmodule // alu_testbench

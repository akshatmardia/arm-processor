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

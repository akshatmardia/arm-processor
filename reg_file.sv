// reg_file takes as input: a 1-bit clk (the clock), a 1-bit write enable called wr_en,
// 32-bit write_data which holds the data to be written, 4-bit write_addr which holds the address
// to be written to, 4-bit read_addr1 and 4-bit read_addr2 which are two addresses to read data from.
// read_data1 and read_data2 are two 32-bit outputs that will hold the data which was read at its corresponding address:
// read_data1 -> read_addr1
// read_data2 -> read_addr2
// The purpose of this module is to act as a 16x32 register file with two read port, one write port,
// synchronous write, and asynchronous read.

module reg_file(input logic clk, wr_en,
	input logic [31:0] write_data
	,input logic [3:0] write_addr
	,input logic [3:0]	read_addr1, read_addr2
	,output logic [31:0] read_data1, read_data2);

	// memory serves as the 16x32 internal registers of the register file
	logic [15:0][31:0] memory;

	// Since the write functionality must be synchronous it should be operate within an always_ff which triggers at the
	// positive edge of the clock.
	// The if statement is important since it ensures that the write enable wr_en is active before it allows registers
	// to be written to.
	// Once it is enabled it writes write_data into the register at write_addr.
	always_ff @(posedge clk) begin
		if (wr_en)
			memory[write_addr] <= write_data;
	end // always_ff

	// read_data1 and read_data2 are set using assign statements outside the always_ff since they must be asynchronous.
	// Each read_data is read from the register at its corresponding read_addr
	assign read_data1 = memory[read_addr1];
	assign read_data2 = memory[read_addr2];

endmodule // reg_file

// reg_file_testbench tests reg_file for the following three cases which, if satisfied, ensure its proper functionality:
//		1. Write data is written into the register file the clock cycle after wr_en is asserted.
//		2. Read data is updated to the register data at an address the same cycle the address was
//			provided. Do this for both read addresses and data outputs.
//		3. Read data is updated to write data at an address the cycle after the address was provided
//			if the write address is the same and wr_en was asserted. Do this for both read addresses and data outputs.
// similarly to reg_file:
// reg_file_testbench takes as input: a 1-bit clk (the clock), a 1-bit write enable called wr_en,
// 32-bit write_data which holds the data to be written, 4-bit write_addr which holds the address
// to be written to, 4-bit read_addr1 and 4-bit read_addr2 which are two addresses to read data from.
// read_data1 and read_data2 are two 32-bit outputs that will hold the data which was read at its corresponding address:
// read_data1 -> read_addr1
// read_data2 -> read_addr2
module reg_file_testbench();


	logic clk, wr_en;
	logic [31:0] write_data;
	logic [3:0] write_addr;
	logic [3:0]	read_addr1, read_addr2;
	logic [31:0] read_data1, read_data2;

	// reg_file dut takes the exact same inputs and outputs described above.
	reg_file dut (.*);

	// Set up the clock.
   parameter CLOCK_PERIOD=100;
   initial begin
	 clk <= 0;
	 forever #(CLOCK_PERIOD/2) clk <= ~clk;
   end

	initial begin
		wr_en <= 0;
		write_data <= 32'b0; write_addr <= 4'b0001; read_addr1 <= 4'b0001; read_addr2 <= 4'b1000; @(posedge clk);

		wr_en <= 1;
		write_data <= 32'b0; write_addr <= 4'b0; @(posedge clk);
		write_data <= {32{1'b1}}; write_addr <= 4'b1111; read_addr1 <= 4'b1111; @(posedge clk);
		write_data <= {{31{1'b0}}, 1'b1}; read_addr1 <= 4'b0; read_addr2 <= 4'b1111; @(posedge clk);

		wr_en <= 0;
		write_data <= {32{1'b1}}; write_addr <= 4'b0; read_addr1 <= 4'b0; @(posedge clk);
		write_data <= 32'b0; write_addr <= 4'b1111; read_addr2 <= 4'b1111; @(posedge clk);

		wr_en <= 1;
		write_data <= {32{1'b1}}; write_addr <= 4'b0; read_addr1 <= 4'b0; @(posedge clk);
		write_data <= 32'b0; write_addr <= 4'b1111; read_addr2 <= 4'b1111; @(posedge clk);

		$stop; // stops the simulation to keep the clock from going to infinity
	end
endmodule // reg_file_testbench

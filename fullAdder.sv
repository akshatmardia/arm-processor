module fullAdder (A, B, cin, sum, cout);

	input logic A, B, cin;
	output logic sum, cout;

	assign sum = A ^ B ^ cin;
	assign cout = A&B | cin & (A^B);

endmodule

module fullAdder_testbench();

	logic A, B, cin, sum, cout;

	fullAdder dut (A, B, cin, sum, cout);

	integer i;
	initial begin

		for (i = 0; i<2**3; i++) begin
			{A, B, cin} = i; #10;
		end //for loop

	end //initial

endmodule

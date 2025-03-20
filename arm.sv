/* arm is the spotlight of the show and contains the bulk of the datapath and control logic. This module is split into two parts, the datapath and control.
*/

// clk - system clock
// rst - system reset
// Instr - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates
// ReadData - data read out of the dmem
// WriteData - data to be written to the dmem
// MemWrite - write enable to allowed WriteDataE to overwrite an existing dmem word
// PCF - the current program count value, goes to imem to fetch instruciton
// ALUResult - result of the ALU operation, sent as address to the dmem

module arm (
    input  logic        clk, rst,
    input  logic [31:0] Instr,
    input  logic [31:0] ReadData,
    output logic [31:0] WriteData,
    output logic [31:0] PCF, ALUResult,
    output logic        MemWrite
);

    // datapath buses and signals
    logic [31:0] PCPrime, PCPlus4F, PCPlus8D, PCHolder; // pc signals
    logic [ 3:0] RA1D, RA2D, RA1E, RA2E;      			  // regfile input addresses
    logic [31:0] RD1D, RD2D, RD1E, RD2E, RD1DPrime, RD2DPrime;                // raw regfile outputs
    logic [ 3:0] ALUFlags;                  				  // alu combinational flag outputs
    logic [31:0] ExtImmD, ExtImmE, SrcAE, SrcBE;        // immediate and alu inputs
    logic [31:0] ResultW;                               // computed or fetched value to be written into regfile or pc
    logic [31:0] InstrF, InstrD;						  		  
    logic [ 3:0] WA3D, WA3E, WA3M, WA3W;
	 logic [31:0] WriteDataE, WriteDataM;
	 logic [31:0] ALUResultE;
	 logic [31:0] ALUOutM, ALUOutW;
	 logic [31:0] ReadDataM, ReadDataW;

    // control signals
    logic PCSrcD, PCSrcE, PCSrcM, PCSrcW;
    logic RegWriteD, RegWriteE, RegWriteM, RegWriteW;
    logic MemtoRegD, MemtoRegE, MemtoRegM, MemtoRegW;
	 logic MemWriteD, MemWriteE, MemWriteM;
	 logic [1:0] ALUControlD, ALUControlE;
	 logic BranchD, BranchE;
    logic ALUSrcD, ALUSrcE;
	 logic FlagWriteD, FlagWriteE;
	 logic [1:0] ImmSrcD, RegSrcD;
	 logic CondExE, BranchTakenE;
	 logic [3:0] CondE;
    logic [3:0] FlagsPrime, FlagsE;
	 
	 // Hazard Unit signals
	 logic Match_1E_M, Match_2E_M, Match_1E_W, Match_2E_W;
	 logic Match_12D_E, ldrstallD, StallF, StallD, FlushD, FlushE;
	 logic PCWrPendingF;
	 logic [1:0] ForwardAE, ForwardBE;


    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------
	 
	 assign InstrF = Instr;
	 assign PCHolder = PCSrcW ? ResultW : PCPlus4F; 			// mux, use either default or newly computed value
    assign PCPrime = BranchTakenE ? ALUResultE : PCHolder;  // mux, for early branch resolution
    assign PCPlus4F = PCF + 'd4;                  				// default value to access next instruction
    assign PCPlus8D = PCPlus4F;             						// value read when reading from reg[15]

    // update the PCF, at rst initialize to 0
    always_ff @(posedge clk) begin
		  // set PC to 0 if reset, keep it as is if StallF, else move forward according to logic
        if (rst) 			 PCF <= '0;
		  else if (StallF) PCF <= PCF;
        else     			 PCF <= PCPrime;
    end

	 // set InstrD to InstrF inside the flip flop for pipelining at DECODE, at rst initialize to 0
    always_ff @(posedge clk) begin
			// flush or reset set all 0, if stall and flush then flush, if stall and !flush keep it as is, else move forward
        if (rst | FlushD) InstrD <= 'b?;
		  else if (StallD & FlushD) InstrD <= 'b?;
		  else if (StallD & ~FlushD) InstrD <= InstrD;
        else     InstrD <= InstrF;
    end
	 
	 assign WA3D = InstrD[15:12];

    // determine the register addresses based on control signals
    // RegSrcD[0] is set if doing a branch instruction
    // RefSrc[1] is set when doing memory instructions
    assign RA1D = RegSrcD[0] ? 4'd15 : InstrD[19:16];
    assign RA2D = RegSrcD[1] ? WA3D  : InstrD[3: 0];

    // Calling the Register File here to access data in the registers at given inputs
    reg_file u_reg_file (
        .clk       (~clk),
        .wr_en     (RegWriteW),
        .write_data(ResultW),
        .write_addr(WA3W),
        .read_addr1(RA1D),
        .read_addr2(RA2D),
        .read_data1(RD1D),
        .read_data2(RD2D)
    );
	 
	 assign RD1DPrime = (RA1D == 'd15) ? PCPlus8D : RD1D;	// substitute the 15th regfile register for PC
	 assign RD2DPrime = (RA2D == 'd15) ? PCPlus8D : RD2D;	// substitute the 15th regfile register for PC
	 
	 always_comb begin
		// two muxes, determines which set of instruction bits are used for the immediate
		if      (ImmSrcD == 'b00) ExtImmD = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
		else if (ImmSrcD == 'b01) ExtImmD = {20'b0, InstrD[11:0]};                  // 12 bit immediate - mem operations
		else 							  ExtImmD = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
	 end

	 // set values inside the flip flop for pipelining at EXECUTE stage, at rst initialize to 0
    always_ff @(posedge clk) begin
        if (rst | FlushE) begin //flush or reset set all 0, else move forward
				RD1E <= '0;
				RD2E <= '0;
				ExtImmE <= '0;
				WA3E <= '0;
				RA1E <= '0;
				RA2E <= '0;
            // Control signals
				PCSrcE <= '0;
				RegWriteE <= '0;
				MemtoRegE <= '0;
			   MemWriteE <= '0;
		   	ALUControlE <= '0;
				BranchE <= '0;
			   ALUSrcE <= '0;
				FlagWriteE <= '0;
				CondE <= '0;
				FlagsE <= '0;
			end
        else begin
				RD1E <= RD1DPrime;
				RD2E <= RD2DPrime;
				RA1E <= RA1D;
				RA2E <= RA2D;
				WA3E <= WA3D;
				ExtImmE <= ExtImmD;
			   // Control Signals
			   PCSrcE <= PCSrcD;
			   RegWriteE <= RegWriteD;
			   MemtoRegE <= MemtoRegD;
			   MemWriteE <= MemWriteD;
			   ALUControlE <= ALUControlD;
			   BranchE <= BranchD;
			   ALUSrcE <= ALUSrcD;
			   FlagWriteE <= FlagWriteD;
				CondE <= InstrD[31:28];
				if (FlagWriteE) FlagsE <= FlagsPrime;
		  end
    end
	 
	 // Muxes to determine forwarding logic
	 // WriteDataE and SrcAE are direct outputs of the register file, wheras SrcBE is chosen between reg file output and the immediate
	 always_comb begin
		if      (ForwardAE == 'b00) SrcAE = RD1E;
		else if (ForwardAE == 'b01) SrcAE = ResultW;
		else 								 SrcAE = ALUOutM;
		
		if      (ForwardBE == 'b00) WriteDataE = RD2E;
		else if (ForwardBE == 'b01) WriteDataE = ResultW;
		else 								 WriteDataE = ALUOutM;
		SrcBE = ALUSrcE ? ExtImmE : WriteDataE;     // determine alu operand to be either from reg file or from immediate
	 end

    // Calling the ALU to determine ALUResult and save flags when needed
    alu u_alu (
        .a          (SrcAE),
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (ALUResultE),
        .ALUFlags   (ALUFlags)
    );
	 
	 // Logic for Flags' and BranchTakenE coming out of Cond Unit
	 always_comb begin
		FlagsPrime = ALUFlags;
		BranchTakenE = (BranchE & CondExE);
	 end

	 // set values inside the flip flop for pipelining at MEMORY, at rst initialize to 0
    always_ff @(posedge clk) begin
        if (rst) begin
					WA3M <= '0;
					ALUOutM <= '0;
					WriteDataM <= '0;
					// Control signals
					PCSrcM <= '0;
					RegWriteM <= '0;
				   MemtoRegM <= '0;
               MemWriteM <= '0;
        end
        else begin
					WA3M <= WA3E;
					ALUOutM <= ALUResultE;
					WriteDataM <= WriteDataE;
					// Control signals
			      PCSrcM <= (PCSrcE & CondExE);
					RegWriteM <= (RegWriteE & CondExE);
				   MemtoRegM <= MemtoRegE;
					MemWriteM <= (MemWriteE & CondExE);
        end
    end
	 
	 // Assigning output signals for the data memory
	 always_comb begin
			ALUResult = ALUOutM;
			WriteData = WriteDataM;
			ReadDataM = ReadData;
			MemWrite = MemWriteM;
	 end

	 // set values inside the flip flop for pipelining at WRITEBACK, at rst initialize to 0;
    always_ff @(posedge clk) begin
        if (rst) begin
					WA3W <= '0;
					ALUOutW <= '0;
					ReadDataW <= '0;
					// Control signals
				   PCSrcW <= '0;
				   RegWriteW <= '0;
				   MemtoRegW <= '0;
        end
        else begin
					WA3W <= WA3M;
					ALUOutW <= ALUOutM;
					ReadDataW <= ReadDataM;
				  // Control signals
				   PCSrcW <= PCSrcM;
				   RegWriteW <= RegWriteM;
				   MemtoRegW <= MemtoRegM;
        end
    end

    // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu


    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are
    ** especially important because they are representative of your processors current state.
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------

    always_comb begin
        casez (InstrD[27:20])

            // ADD (Imm or Reg)
            8'b00?_0100_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
                PCSrcD    = 0;
                MemtoRegD = 0;
                MemWriteD = 0;
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;
                ALUControlD = 'b00;
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // SUB (Imm or Reg)
            8'b00?_0010_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrcD    = 0;
                MemtoRegD = 0;
                MemWriteD = 0;
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;
                ALUControlD = 'b01;
					 FlagWriteD = 0;
					 BranchD = 0;
            end

				// CMP/SUBS (Imm or Reg)
            8'b00?_0010_1 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrcD    = 0;
                MemtoRegD = 0;
                MemWriteD = 0;
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;
                ALUControlD = 'b01;
					 FlagWriteD = 1;
					 BranchD = 0;
            end

            // AND
            8'b000_0000_0 : begin
                PCSrcD    = 0;
                MemtoRegD = 0;
                MemWriteD = 0;
                ALUSrcD   = 0;
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b10;
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // ORR
            8'b000_1100_0 : begin
                PCSrcD    = 0;
                MemtoRegD = 0;
                MemWriteD = 0;
                ALUSrcD   = 0;
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD  = 'b00;    // doesn't matter
                ALUControlD = 'b11;
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // LDR
            8'b010_1100_1 : begin
                PCSrcD    = 0;
                MemtoRegD = 1;
                MemWriteD = 0;
                ALUSrcD   = 1;
                RegWriteD = 1;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01;
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // STR
            8'b010_1100_0 : begin
                PCSrcD    = 0;
                MemtoRegD = 0; // doesn't matter
                MemWriteD = 1;
                ALUSrcD  = 1;
                RegWriteD = 0;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01;
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // B
            8'b1010_???? : begin
				    PCSrcD    = 0;
				    MemtoRegD = 0;
				    MemWriteD = 0;
				    ALUSrcD   = 1;
				    RegWriteD = 0;
				    RegSrcD   = 'b01;
				    ImmSrcD   = 'b10;
				    ALUControlD = 'b00;  // do an add
				    FlagWriteD = 0;
				    BranchD = 1;
				end


			default: begin
				    PCSrcD    = 0;
				    MemtoRegD = 0; // doesn't matter
				    MemWriteD = 0;
				    ALUSrcD   = 0;
				    RegWriteD = 0;
				    RegSrcD   = 'b00;
				    ImmSrcD   = 'b00;
				    ALUControlD = 'b00;  // do an add
				    FlagWriteD = 0;
				    BranchD = 0;
			   end
        endcase
		  
		  case (CondE)
					// N Z C V
					// Unconditional
					4'b1110: CondExE = 1;

					// Equal
					4'b0000: CondExE = FlagsE[2];

					// Not Equal
					4'b0001: CondExE = ~FlagsE[2];

					// Greater or Equal
					4'b1010: CondExE = ~(FlagsE[3] ^ FlagsE[0]);

					// Greater
					4'b1100: CondExE = ~(FlagsE[3] ^ FlagsE[0]) & ~FlagsE[2];

					// Less or Equal
					4'b1101: CondExE = (FlagsE[3] ^ FlagsE[0]) | FlagsE[2];

					// Less
					4'b1011: CondExE = (FlagsE[3] ^ FlagsE[0]);

					default: begin
					CondExE = 0;
					end
				endcase
    end
	 
	 //-------------------------------------------------------------------------------
    //                                      Hazard Unit
    //-------------------------------------------------------------------------------
	 // Assigning logic for the Hazard Unit in case some values are unavailable at the point in time when they are required,
	 // in which case we stall the processor and/or flush the existing values. Also handles the forwarding logic.
	 always_comb begin
	 
		Match_1E_M = (RA1E == WA3M);
		Match_2E_M = (RA2E == WA3M);
		 
		Match_1E_W = (RA1E == WA3W);
		Match_2E_W = (RA2E == WA3W);
		 
		Match_12D_E = (RA1D == WA3E) | (RA2D == WA3E);
		ldrstallD = Match_12D_E & MemtoRegE;
		 
		PCWrPendingF = PCSrcD | PCSrcE | PCSrcM;
		 
		StallF = ldrstallD | PCWrPendingF;
		FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
		FlushE = ldrstallD | BranchTakenE;
		StallD = ldrstallD;
		if      (Match_1E_M & RegWriteM) ForwardAE = 'b10;
		else if (Match_1E_W & RegWriteW) ForwardAE = 'b01;
		else 								 		ForwardAE = 'b00;
		
		if      (Match_2E_M & RegWriteM) ForwardBE = 'b10;
		else if (Match_2E_W & RegWriteW) ForwardBE = 'b01;
		else 								 		ForwardBE = 'b00;
	 end

endmodule

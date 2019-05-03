// arm_pipelined.sv
// David_Harris@hmc.edu, Sarah.Harris@unlv.edu 4 January 2014
// Pipelined implementation of a subset of ARMv4

// 16 32-bit registers
// Data-processing instructions
//   ADD, SUB, AND, ORR
//   OP <Cond> <S> <Rd>, <Rn>, #immediate
//   OP <Cond> <S> <Rd>, <Rn>, <Rm>
//    Rd <- <Rn> OP <Rm>	    if (S) Update Status Flags
//    Rd <- <Rn> OP immediate	if (S) Update Status Flags
//   Instr[31:28] = Cond
//   Instr[27:26] = Op = 00
//   Instr[25:20] = Funct
//                  [25]:    1 for immediate, 0 for register
//                  [24:21]: 0100 (ADD) / 0010 (SUB) /
//                           0000 (AND) / 1100 (ORR)
//                  [20]:    S (1 = update CPSR status Flags)
//   Instr[19:16] = Rn
//   Instr[15:12] = Rd
//   Instr[11:8]  = 0000
//   Instr[7:0]   = immed_8  (for #immediate type) / 
//                  0000<Rm> (for register type)
//   
// Load/Store instructions
//   LDR, STR
//   OP <Rd>, <Rn>, #offset
//    LDR: Rd <- Mem[<Rn>+offset]
//    STR: Mem[<Rn>+offset] <- Rd
//   Instr[31:28] = Cond
//   Instr[27:26] = Op = 01 
//   Instr[25:20] = Funct
//                  [25]:    0 (A)
//                  [24:21]: 1100 (P/U/B/W)
//                  [20]:    L (1 for LDR, 0 for STR)
//   Instr[19:16] = Rn
//   Instr[15:12] = Rd
//   Instr[11:0]  = imm (zero extended)
//
// Branch instruction (PC <= PC + offset, PC holds 8 bytes past Branch Instr)
//   B
//   OP <target>
//    PC <- PC + 8 + imm << 2
//   Instr[31:28] = Cond
//   Instr[27:25] = Op = 10
//   Instr[25:24] = Funct
//                  [25]: 1 (Branch)
//                  [24]: 0 (link)
//   Instr[23:0]  = offset (sign extend, shift left 2)
//   Note: no Branch delay slot on ARM
//
// Other:
//   R15 reads as PC+8
//   Conditional Encoding
//    Cond  Meaning                       Flag
//    0000  Equal                         Z = 1
//    0001  Not Equal                     Z = 0
//    0010  Carry Set                     C = 1
//    0011  Carry Clear                   C = 0
//    0100  Minus                         N = 1
//    0101  Plus                          N = 0
//    0110  Overflow                      V = 1
//    0111  No Overflow                   V = 0
//    1000  Unsigned Higher               C = 1 & Z = 0
//    1001  Unsigned Lower/Same           C = 0 | Z = 1
//    1010  Signed greater/equal          N = V
//    1011  Signed less                   N != V
//    1100  Signed greater                N = V & Z = 0
//    1101  Signed less/equal             N != V | Z = 1
//    1110  Always                        any
//
// run 380
// Expect simulator to print "Simulation succeeded"
// when the value 7 is written to address 100 (0x64)‭

module testbench();

   logic        clk;
   logic        reset;
   logic [31:0] WriteData, DataAdr;
   logic        MemWrite;

   // instantiate device to be tested
   top dut (clk, reset, WriteData, DataAdr, MemWrite);
   
   // initialize test
   initial
     begin
	reset <= 1; # 22; reset <= 0;
     end

   // generate clock to sequence tests
   always
     begin
	clk <= 1; # 5; clk <= 0; # 5;
     end

endmodule // testbench

module arm (input  logic        clk, reset,
(* mark_debug = "true" *)            output logic [31:0] PCF,
(* mark_debug = "true" *)            input  logic [31:0] InstrF,
            output logic        MemWriteM,
            output logic [31:0] ALUOutM, WriteDataM,
            input  logic [31:0] ReadDataM,
			output logic 		MemStrobe,
			input  logic 		PReady);
   
   logic [2:0] 			RegSrcD, shifterFlagsE;
   logic [1:0] 			ImmSrcD;
   logic 			ALUSrcE, BranchTakenE, MemtoRegW,
				PCSrcW, RegWriteW, srcBtoRegE, carryE;
   logic [3:0] 			ALUFlagsE, ALUControlE;
   logic [4:0]			shamtE;
   logic [31:0] 		InstrD;
   logic 			RegWriteM, MemtoRegE, PCWrPendingF;
   logic [1:0] 			ForwardAE, ForwardBE, shiftTypeE;
   logic 			StallF, StallD, FlushD, FlushE;
   logic 			Match_1E_M, Match_1E_W, 
				Match_2E_M, Match_2E_W, 
				Match_12D_E;
   
   controller c (clk, reset, InstrD[31:5], ALUFlagsE, shifterFlagsE,
		 RegSrcD, ImmSrcD, shiftTypeE, shamtE,
		 ALUSrcE, BranchTakenE, srcBtoRegE, carryE, ALUControlE,
		 MemWriteM,
		 MemtoRegW, PCSrcW, RegWriteW,
		 RegWriteM, MemtoRegE, PCWrPendingF,
		 FlushE, MemStrobe, PReady);
   datapath dp (clk, reset, 
		RegSrcD, ImmSrcD, shiftTypeE, shamtE,
		ALUSrcE, BranchTakenE, ALUControlE,
		MemtoRegW, PCSrcW, RegWriteW, srcBtoRegE, carryE,
		PCF, InstrF, InstrD,
		ALUOutM, WriteDataM, ReadDataM,
		ALUFlagsE, shifterFlagsE,
		Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W, 
		Match_12D_E,
		ForwardAE, ForwardBE, StallF, StallD, FlushD, PReady);   
   hazard h (clk, reset, Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W, 
	     Match_12D_E,
             RegWriteM, RegWriteW, BranchTakenE, MemtoRegE,
             PCWrPendingF, PCSrcW,
             ForwardAE, ForwardBE,
             StallF, StallD, FlushD, FlushE);
   
endmodule // arm

module controller (input  logic         clk, reset,
                   input  logic [31:5] InstrD,
				   input  logic [3:0]   ALUFlagsE,
				   input  logic [2:0]   shifterFlagsE,
                   output logic [2:0]   RegSrcD, 
				   output logic [1:0]   ImmSrcD, shiftTypeE,
				   output logic [4:0]   shamtE,
                   output logic         ALUSrcE, BranchTakenE, srcBtoRegE, carryE,
                   output logic [3:0]   ALUControlE,
                   output logic         MemWriteM,
                   output logic         MemtoRegW, PCSrcW, RegWriteW,
                   // hazard interface
                   output logic         RegWriteM, MemtoRegE,
                   output logic         PCWrPendingF,
                   input  logic         FlushE,
				   //Memory interfaces
				   output logic			MemStrobe,
				   input  logic			PReady);

   logic [11:0]				controlsD;
   logic 				CondExE, ALUOpD;
   logic [3:0] 				ALUControlD;
   logic 				ALUSrcD, srcBtoRegD;
   logic 				MemtoRegD, MemtoRegM;
   logic 				RegWriteD, RegWriteE, RegWriteGatedE;
   logic 				MemWriteD, MemWriteE, MemWriteGatedE;
   logic 				BranchD, BranchE;
   logic				cFromShifterD, nzFromShifterD, cFromShifterE, nzFromShifterE;
   logic [1:0] 				FlagWriteD, FlagWriteE, shiftTypeD, Op;
   logic 				PCSrcD, PCSrcE, PCSrcM, CondExM;
   logic [3:0] 				FlagsE, FlagsNextE, CondE;
   logic [4:0]			shamtD;

   logic [5:0] Funct;
   logic [6:0] shiftInfoD;
   assign Funct = InstrD[25:20];

   // Decode stage   
   always_comb
     casex(InstrD[27:26])
       //Data operations.
       2'b00: controlsD = 12'b000_00_0_0_0_0_0_0_1 | 
					{5'd0, Funct[5], 6'd0} | //Immediate?
               {7'd0, (Funct[4] & Funct[3] & ~Funct[2] & Funct[1]), 4'd0} | //Write srcb to reg?
               {8'd0, ~(Funct[4] & ~Funct[3]), 3'd0}; //Write to register, or just update fields?
       
       // LDR/STR
       2'b01: controlsD = 12'b000_01_1_1_0_0_0_0_0 |
				   {1'b0, ~Funct[0], 6'd0, Funct[0], ~Funct[0], 2'd0}; //Funct[0] indicates LDR
              
       // B/BL
       2'b10: controlsD = 12'b001_10_1_0_0_0_0_1_0 |
					{Funct[4], 7'd0, Funct[4], 3'd0}; //Account for BL
       
       default:               controlsD = 12'bx;          // unimplemented
     endcase

   // bits:  3,2,1,1,1,1,1,1 = 11
   assign {RegSrcD, ImmSrcD, ALUSrcD, MemtoRegD, srcBtoRegD, 
           RegWriteD, MemWriteD, BranchD, ALUOpD} = controlsD; 
   assign shiftInfoD = InstrD[11:5];
   assign Op = InstrD[27:26];
   assign shamtD = {shiftInfoD[6:3], shiftInfoD[2] & ~Funct[5]} & {5{(~Op[1]) & (~Op[0])}}; //Last bit always 0 for immediates, only non-zero value for data operations
   assign shiftTypeD = shiftInfoD[1:0] | {Funct[5], Funct[5]}; //Always 11 (ROR) for immediates.
   
   //Data ops other than add, adc, sub, sbc, rsb, rsc, cmp, cmn use the shifter values.
	//Could simplify by moving below setting ALUControl and using 0?0? of ALUControl.
	//	Probably should. But I'm lazy and don't want to.
	assign cFromShifterD = {(~Op[1]) & (~Op[0])} & ~( (~Funct[4] & (Funct[3] | Funct[2])) | (Funct[4] & ~Funct[3] & Funct[2]));
	//Moves and shifts don't pass through the ALU, and want to use n/z from the shifter.
	assign nzFromShifterD = {(~Op[1]) & (~Op[0])} & Funct[4] & Funct[3] & ~Funct[2] & Funct[1];

	always_comb
     if (ALUOpD) begin                 // which DP Instr?
	    case(InstrD[24:21]) 
  	      4'b0100: ALUControlD = 4'b0000; // ADD
  	      4'b0101: ALUControlD = 4'b0100; // ADC
      	  4'b0010: ALUControlD = 4'b0001; // SUB
      	  4'b0110: ALUControlD = 4'b0101; // SBC
          4'b0000: ALUControlD = 4'b0010; // AND
		  4'b1000: ALUControlD = 4'b0010; // TST, just and
      	  4'b1100: ALUControlD = 4'b0011; // ORR
		  4'b0001: ALUControlD = 4'b0111; // EOR
		  4'b1001: ALUControlD = 4'b0111; // TEQ, just xor
    	  4'b1110: ALUControlD = 4'b0110; // BIC, and w/ inverted b
    	  4'b1011: ALUControlD = 4'b0000; // CMN, just add
    	  4'b1010: ALUControlD = 4'b0001; // CMP, just subtract
		  4'b1111: ALUControlD = 4'b1000; // MVN, do ~b
      	  default: ALUControlD = 4'bx;   // unimplemented
		endcase
		// update flags if S bit is set 
		// (C & V only updated for arith instructions)
		FlagWriteD[1]      = Funct[0]; // FlagW[1] = S-bit
		// FlagW[0] = S-bit & (ADD | SUB) (ADD/SUB indicated by 0?0? to account for add/adc/sub/sbc)
		FlagWriteD[0]      = Funct[0] & 
			~(ALUControlD[3] | ALUControlD[1]); 
     end else begin
		ALUControlD = 4'b0000; // add for non-DP instructions
		FlagWriteD      = 2'b00; // don't update Flags
     end

   assign PCSrcD       = (((InstrD[15:12] == 4'b1111) & RegWriteD) | BranchD);
   
   // Execute stage
   flopenrc #(7) flushedregsE(clk, reset, ~PReady, FlushE, 
                            {FlagWriteD, BranchD, MemWriteD, 
			     RegWriteD, PCSrcD, MemtoRegD},
                            {FlagWriteE, BranchE, MemWriteE, 
			     RegWriteE, PCSrcE, MemtoRegE});
   flopenr #(15)  regsE(clk, reset, ~PReady,
                     {ALUSrcD, srcBtoRegD, ALUControlD, shamtD, shiftTypeD, cFromShifterD, nzFromShifterD},
                     {ALUSrcE, srcBtoRegE, ALUControlE, shamtE, shiftTypeE, cFromShifterE, nzFromShifterE});
   
   flopenr  #(4) condregE(clk, reset, ~PReady, InstrD[31:28], CondE);
   flopenr  #(4) flagsreg(clk, reset, ~PReady, FlagsNextE, FlagsE);
   assign carryE = FlagsE[1];

   // write and Branch controls are conditional
   conditional Cond (CondE, FlagsE, ALUFlagsE, shifterFlagsE, cFromShifterE, nzFromShifterE,
			 FlagWriteE, CondExE, FlagsNextE);
   assign BranchTakenE    = BranchE & CondExE;
   assign RegWriteGatedE  = RegWriteE & CondExE;
   assign MemWriteGatedE  = MemWriteE & CondExE;
   assign PCSrcGatedE     = PCSrcE & CondExE;
   
   // Memory stage
   flopenr #(5) regsM(clk, reset, ~PReady,
                    {MemWriteGatedE, MemtoRegE, RegWriteGatedE, PCSrcGatedE, CondExE},
                    {MemWriteM, MemtoRegM, RegWriteM, PCSrcM, CondExM});
   assign MemStrobe = (MemtoRegM | MemWriteM) & CondExM;
   
   // Writeback stage
   flopenr #(3) regsW(clk, reset, ~PReady,
                    {MemtoRegM, RegWriteM, PCSrcM},
                    {MemtoRegW, RegWriteW, PCSrcW});
   
   // Hazard Prediction
   assign PCWrPendingF = PCSrcD | PCSrcE | PCSrcM;

endmodule // controller

module conditional (input  logic [3:0] Cond,
                    input  logic [3:0] Flags,
                    input  logic [3:0] ALUFlags,
					input  logic [2:0] shifterFlags,
					input  logic cFromShifter, nzFromShifter,
                    input  logic [1:0] FlagsWrite,
                    output logic       CondEx,
                    output logic [3:0] FlagsNext);
   
   logic 	   neg, zero, carry, overflow, ge, cFlagToUse;
   logic [1:0] nzFlagsToUse;
   
   assign {neg, zero, carry, overflow} = Flags;
   assign ge = (neg == overflow);
   
   always_comb
     case(Cond)
       4'b0000: CondEx = zero;             // EQ
       4'b0001: CondEx = ~zero;            // NE
       4'b0010: CondEx = carry;            // CS
       4'b0011: CondEx = ~carry;           // CC
       4'b0100: CondEx = neg;              // MI
       4'b0101: CondEx = ~neg;             // PL
       4'b0110: CondEx = overflow;         // VS
       4'b0111: CondEx = ~overflow;        // VC
       4'b1000: CondEx = carry & ~zero;    // HI
       4'b1001: CondEx = ~(carry & ~zero); // LS
       4'b1010: CondEx = ge;               // GE
       4'b1011: CondEx = ~ge;              // LT
       4'b1100: CondEx = ~zero & ge;       // GT
       4'b1101: CondEx = ~(~zero & ge);    // LE
       4'b1110: CondEx = 1'b1;             // Always
       default: CondEx = 1'bx;             // undefined
     endcase
   
   assign nzFlagsToUse = nzFromShifter ? shifterFlags[2:1] : ALUFlags[3:2];
   assign cFlagToUse = cFromShifter ? shifterFlags[0] : ALUFlags[1];
   
   assign FlagsNext[3:1] = (FlagsWrite[1] & CondEx) ? 
			   {nzFlagsToUse, cFlagToUse} : Flags[3:1];
   assign FlagsNext[0] = (FlagsWrite[0] & CondEx) ? 
			   ALUFlags[0] : Flags[0];

endmodule // conditional

module datapath (input  logic        clk, reset,
                 input  logic [2:0]  RegSrcD,
				 input  logic [1:0]  ImmSrcD, shiftTypeE,
				 input  logic [4:0]  shamtE,
                 input  logic        ALUSrcE, BranchTakenE,
                 input  logic [3:0]  ALUControlE, 
                 input  logic        MemtoRegW, PCSrcW, RegWriteW, srcBtoRegE, carryE,
                 output logic [31:0] PCF,
                 input  logic [31:0] InstrF,
                 output logic [31:0] InstrD,
                 output logic [31:0] DataResultM, WriteDataM,
                 input  logic [31:0] ReadDataM,
                 output logic [3:0]  ALUFlagsE,
				 output logic [2:0]  shifterFlagsE,
                 // hazard logic
                 output logic        Match_1E_M, Match_1E_W, 
		 output logic        Match_2E_M, Match_2E_W, Match_12D_E,
                 input  logic [1:0]  ForwardAE, ForwardBE,
                 input  logic        StallF, StallD, FlushD, PReady);
   
   logic [31:0] 		     PCPlus4F, PCnext1F, PCnextF;
   logic [31:0] 		     PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W;   
   logic [31:0] 		     ExtImmD, rd1D, rd2D, PCPlus8D;
   logic [31:0] 		     rd1E, rd2E, ExtImmE, SrcAE, SrcBE, Op2E;
   logic [31:0] 		     WriteDataE, ALUResultE, DataResultE;
   logic [31:0] 		     ReadDataW, DataResultW, ResultW;
   logic [3:0] 			     RA1D, RA2D, RA3D, RA1E, RA2E;
   logic [31:0] 		     RA4D;   
   logic [3:0] 			     WA3E, WA3M, WA3W;
   logic 			     Match_1D_E, Match_2D_E;
   logic [2:0] 			     RegSrcE, RegSrcM, RegSrcW;
      
   // Fetch stage
   mux2 #(32) pcnextmux (PCPlus4F, ResultW, PCSrcW, PCnext1F);
   mux2 #(32) branchmux (PCnext1F, ALUResultE, BranchTakenE, PCnextF);
   flopenr #(32) pcreg (clk, reset, (~StallF & ~PReady), PCnextF, PCF);
   adder #(32) pcadd (PCF, 32'h4, PCPlus4F);
   
   // Decode Stage
   assign PCPlus8D = PCPlus4F; // skip register
   flopenrc #(32) instrreg (clk, reset, (~StallD & ~PReady), FlushD, InstrF, InstrD);
   flopenrc #(32) pcadd4d (clk, reset, (~StallD & ~PReady), FlushD, PCPlus4F, PCPlus4D);
   mux2 #(4)   ra1mux (InstrD[19:16], 4'b1111, RegSrcD[0], RA1D);
   mux2 #(4)   ra2mux (InstrD[3:0], InstrD[15:12], RegSrcD[1], RA2D);
   mux2 #(4)   ra3mux (WA3W, 4'hE, RegSrcW[2], RA3D);
   mux2 #(32)  ra4mux (ResultW, PCPlus4W, RegSrcW[2], RA4D);   
   regfile     rf (clk, RegWriteW, RA1D, RA2D,
                   RA3D, RA4D, PCPlus8D, 
                   rd1D, rd2D); 
   extend      ext (InstrD[23:0], ImmSrcD, ExtImmD);
   
   // Execute Stage
   flopenr #(32) rd1reg (clk, reset, ~PReady, rd1D, rd1E);
   flopenr #(32) rd2reg (clk, reset, ~PReady, rd2D, rd2E);
   flopenr #(32) immreg (clk, reset, ~PReady, ExtImmD, ExtImmE);
   flopenr #(4)  wa3ereg (clk, reset, ~PReady, InstrD[15:12], WA3E);
   flopenr #(4)  ra1reg (clk, reset, ~PReady, RA1D, RA1E);
   flopenr #(4)  ra2reg (clk, reset, ~PReady, RA2D, RA2E);
   flopenr #(32) pcadd4e (clk, reset, ~PReady, PCPlus4D, PCPlus4E);
   flopenr #(3)  regsrce (clk, reset, ~PReady, RegSrcD, RegSrcE);
   mux3 #(32)  byp1mux (rd1E, ResultW, DataResultM, ForwardAE, SrcAE);
   mux3 #(32)  byp2mux (rd2E, ResultW, DataResultM, ForwardBE, WriteDataE);
   mux2 #(32)  srcbmux (WriteDataE, ExtImmE, ALUSrcE, SrcBE);
   basicShifter shifter (SrcBE, shiftTypeE, shamtE, Op2E, shifterFlagsE);
   alu         alu (SrcAE, Op2E, ALUControlE, carryE, ALUResultE, ALUFlagsE);
   mux2 #(32)  resmuxE (ALUResultE, Op2E, srcBtoRegE, DataResultE); 
   
   // Memory Stage
   flopenr #(32) aluresreg (clk, reset, ~PReady, DataResultE, DataResultM);
   flopenr #(32) wdreg (clk, reset, ~PReady, WriteDataE, WriteDataM);
   flopenr #(4)  wa3mreg (clk, reset, ~PReady, WA3E, WA3M);
   flopenr #(32) pcadd4m (clk, reset, ~PReady, PCPlus4E, PCPlus4M);
   flopenr #(3)  regsrcm (clk, reset, ~PReady, RegSrcE, RegSrcM);
   
   // Writeback Stage
   flopenr #(32) aluoutreg (clk, reset, ~PReady, DataResultM, DataResultW);
   flopenr #(32) rdreg (clk, reset, ~PReady, ReadDataM, ReadDataW);
   flopenr #(4)  wa3wreg (clk, reset, ~PReady, WA3M, WA3W);
   flopenr #(32) pcadd4w (clk, reset, ~PReady, PCPlus4M, PCPlus4W);
   flopenr #(3)  regsrcw (clk, reset, ~PReady, RegSrcM, RegSrcW);
   mux2 #(32)  resmux (DataResultW, ReadDataW, MemtoRegW, ResultW);
   
   // hazard comparison
   eqcmp #(4) m0 (WA3M, RA1E, Match_1E_M);
   eqcmp #(4) m1 (WA3W, RA1E, Match_1E_W);
   eqcmp #(4) m2 (WA3M, RA2E, Match_2E_M);
   eqcmp #(4) m3 (WA3W, RA2E, Match_2E_W);
   eqcmp #(4) m4a (WA3E, RA1D, Match_1D_E);
   eqcmp #(4) m4b (WA3E, RA2D, Match_2D_E);
   assign Match_12D_E = Match_1D_E | Match_2D_E;
   
endmodule // datapath

module hazard (input  logic       clk, reset,
               input  logic       Match_1E_M, Match_1E_W, 
	       input  logic       Match_2E_M, Match_2E_W, Match_12D_E,
               input  logic       RegWriteM, RegWriteW,
               input  logic       BranchTakenE, MemtoRegE,
               input  logic       PCWrPendingF, PCSrcW,
               output logic [1:0] ForwardAE, ForwardBE,
               output logic       StallF, StallD,
               output logic       FlushD, FlushE);

   logic 			  ldrStallD;

   // forwarding logic
   always_comb begin
      if (Match_1E_M & RegWriteM)      ForwardAE = 2'b10;
      else if (Match_1E_W & RegWriteW) ForwardAE = 2'b01;
      else                             ForwardAE = 2'b00;
      
      if (Match_2E_M & RegWriteM)      ForwardBE = 2'b10;
      else if (Match_2E_W & RegWriteW) ForwardBE = 2'b01;
      else                             ForwardBE = 2'b00;
   end
   
   // stalls and flushes
   // Load RAW
   //   when an instruction reads a register loaded by the previous,
   //   stall in the decode stage until it is ready
   // Branch hazard
   //   When a branch is taken, flush the incorrectly fetched instrs
   //   from decode and execute stages
   // PC Write Hazard
   //   When the PC might be written, stall all following instructions
   //   by stalling the fetch and flushing the decode stage
   // when a stage stalls, stall all previous and flush next
   
   assign ldrStallD = Match_12D_E & MemtoRegE;
   
   assign StallD = ldrStallD;
   assign StallF = ldrStallD | PCWrPendingF; 
   assign FlushE = ldrStallD | BranchTakenE; 
   assign FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
   
endmodule // hazard

module regfile (input  logic        clk, 
		input  logic        we3, 
		input  logic [3:0]  ra1, ra2, wa3, 
		input  logic [31:0] wd3, r15,
		output logic [31:0] rd1, rd2);
   
   logic [31:0] 		    rf[14:0];

   // three ported register file
   // read two ports combinationally
   // write third port on falling edge of clock (midcycle)
   //   so that writes can be read on same cycle
   // register 15 reads PC+8 instead

   always_ff @(negedge clk)
     if (we3) rf[wa3] <= wd3;	

   assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
   assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];

endmodule // regfile

module extend (input  logic [23:0] Instr,
               input  logic [1:0]  ImmSrc,
               output logic [31:0] ExtImm);
   
   always_comb
     case(ImmSrc) 
       2'b00:   ExtImm = {24'b0, Instr[7:0]};  // 8-bit unsigned immediate
       2'b01:   ExtImm = {20'b0, Instr[11:0]}; // 12-bit unsigned immediate 
       2'b10:   ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00}; // Branch
       default: ExtImm = 32'bx; // undefined
     endcase             

endmodule // extend

module alu (input  logic [31:0] a, b,
            input  logic [3:0]  ALUControl,
			input  logic		carry_in,
			output logic [31:0] Result,
            output logic [3:0]  ALUFlags);
   
   logic 			neg, zero, carry, overflow;
   logic [31:0] 		condinvb;
   logic [32:0] 		sum;
   logic [31:0]     carrycompensator;
   
   always_comb
     casex ({ALUControl[2], ALUControl[0], carry_in})
        3'b0??: carrycompensator = 32'd0; //ALUControl[2] specifies ADC or SBC.
        3'b100: carrycompensator = 32'd0; //Adding, carry is 0
        3'b101: carrycompensator = 32'd1; //Adding, carry is 1
        3'b110: carrycompensator = -32'd1;//Subtracting, carry is 0 
        3'b111: carrycompensator = 32'd0; //Subtracting, carry is 0 
     endcase
   
   assign condinvb = ALUControl[0] ? ~b : b;
   assign sum = a + condinvb + ALUControl[0] + carrycompensator;
   

   always_comb
     casex (ALUControl[3:0])
       4'b0?0?: Result = sum;
       4'b0010: Result = a & b;
	   4'b0110: Result = a & ~b;
       4'b0011: Result = a | b;
	   4'b0111: Result = a ^ b;
	   4'b1000: Result = ~b;
     endcase
   
   assign neg      = Result[31];
   assign zero     = (Result == 32'b0);
   assign carry    = (ALUControl[1] == 1'b0) & sum[32];
   assign overflow = (ALUControl[1] == 1'b0) & 
                     ~(a[31] ^ b[31] ^ ALUControl[0]) & 
                     (a[31] ^ sum[31]); 
   assign ALUFlags    = {neg, zero, carry, overflow};
   
endmodule // alu

module basicShifter (input logic [31:0] src,
					 input logic [1:0] shiftType,
					 input logic [4:0] shamt,
					output logic [31:0] out,
					output logic [2:0] shifterFlags);
	//Make some wires with an extra bit on each side to easily handle carries.
	logic signed [33:0] ext_src;
	assign ext_src = {shiftType[1] & src[31], src, 1'b0}; //If doing ASR, we want MSB the same as orig MSB
	logic [63:0] super_ext_src;
	assign super_ext_src = {src, src}; //Make "rotating" easier (though very inefficient) 
	logic [33:0] ext_out;
	logic [63:0] ror_out;
	assign ror_out = super_ext_src >> shamt;
	
	
	always_comb
		case (shiftType[1:0])
			2'b00: begin
					ext_out = ext_src << shamt; //Shift left
					shifterFlags[0] = ext_src[33];
					out = ext_out[32:1];
					end
			2'b01: begin 
					ext_out = ext_src >> shamt; //Logical shift right
					shifterFlags[0] = ext_src[0];
					out = ext_out[32:1];
					end
			2'b10: begin 
					ext_out = ext_src >>> shamt; //Arithmetic shift right
					shifterFlags[0] = ext_src[0];
					out = ext_out[32:1];
					end
			2'b11: begin 
					ext_out = {1'b0, ror_out[31:0], ror_out[31]}; //ROR
					shifterFlags[0] = ext_src[0];
					out = ext_out[32:1];
					end
		endcase
		
	assign shifterFlags[1] = (out == 32'b0);
	assign shifterFlags[2] = out[31];
endmodule //basicShifter


module adder #(parameter WIDTH=8)
   (input  logic [WIDTH-1:0] a, b, output logic [WIDTH-1:0] y);
   
   assign y = a + b;

endmodule // adder

module flopenr #(parameter WIDTH = 8)
   (input  logic             clk, reset, en,
    input  logic [WIDTH-1:0] d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) q <= d;

endmodule // flopenr

module flopr #(parameter WIDTH = 8)
   (input  logic             clk, reset,
    input  logic [WIDTH-1:0] d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       q <= d;

endmodule // flopr

module flopenrc #(parameter WIDTH = 8)
   (input  logic             clk, reset, en, clear,
    input  logic [WIDTH-1:0] d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) 
       if (clear) q <= 0;
       else       q <= d;

endmodule // flopenrc

module floprc #(parameter WIDTH = 8)
   (input  logic             clk, reset, clear,
    input  logic [WIDTH-1:0] d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       
       if (clear) q <= 0;
       else       q <= d;

endmodule // floprc

/*module mux2 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, 
    input  logic             s, 
    output logic [WIDTH-1:0] y);

   assign y = s ? d1 : d0; 

endmodule // mux2*/

module mux3 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, d2,
    input  logic [1:0]       s, 
    output logic [WIDTH-1:0] y);

   assign y = s[1] ? d2 : (s[0] ? d1 : d0); 

endmodule // mux3

module eqcmp #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] a, b,
    output logic             y);

   assign y = (a == b); 

endmodule // eqcmp


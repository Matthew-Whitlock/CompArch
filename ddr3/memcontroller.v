module MemControl (PReady, MStrobe, MemWrite, clk, reset);
   
   input MStrobe;   // Strobe initiation
   input MemWrite;  // Write
   //input Trigger;   // Trigger when Read/Write from SDRAM
   input clk;       // clk
   input reset;     // reset

   output PReady;   // signal done to processor
   //output DRW;      // Read/Write signal to SDDRAM
   //output AXIstart; // Start of AXI bus transaction

   wire [7:0] WSCLoadVal;
   wire       CtrSig;
   wire       LdCtr;
   wire       Trigger;  // create as input when using ELVIS III
    
   reg [2:0] OutputLogic;   
   
   parameter [2:0] 
     Idle       = 3'b000,
     ReadStart  = 3'b011,
     ReadMem    = 3'b001,
     ReadData   = 3'b010,
     WriteStart = 3'b110,
     WriteMem   = 3'b100,
     WriteData  = 3'b111; 

   reg [2:0]  CURRENT_STATE;
   reg [2:0]  NEXT_STATE;

   // wait state = 15 cycles
   assign WSCLoadVal = 8'hF;
   wait_state WaitStateCtr (LdCtr, WSCLoadVal, Trigger, clk);

   assign {PReady, LdCtr} = OutputLogic;

   always @(negedge clk)
     begin
	if (reset == 1'b1)	
	  CURRENT_STATE <= Idle;
	else
	  CURRENT_STATE <= NEXT_STATE;
     end
   
   always @(CURRENT_STATE or MStrobe or MemWrite or Trigger)
     begin
 	case(CURRENT_STATE)
	  Idle:
	    begin
	       if (MStrobe == 1'b0)
		 NEXT_STATE <= Idle;
	       else if (MemWrite == 1'b1)
		 NEXT_STATE <= WriteStart;
	       else if (MemWrite == 1'b0)
		 NEXT_STATE <= ReadStart;
	       else
		 NEXT_STATE <= Idle;
	       OutputLogic = 2'b01;
	    end // case: Idle
	  ReadStart:
	    begin
               NEXT_STATE <= ReadMem;
               OutputLogic = 2'b10;
            end
	  ReadMem:
	    begin
	       if (Trigger == 1'b0)
		 NEXT_STATE <= ReadMem;
	       else 
		 NEXT_STATE <= ReadData;
	       OutputLogic = 2'b10;
	    end
	  ReadData:
	    begin
	       NEXT_STATE <= Idle;
	       OutputLogic = 2'b10;
	    end
	  WriteStart:
	    begin
	      NEXT_STATE <= WriteMem;
	      OutputLogic = 2'b10;
	    end
	  WriteMem:
	    begin
	       if (Trigger == 1'b0)
		 NEXT_STATE <= WriteMem;
	       else 
		 NEXT_STATE <= WriteData;
	       OutputLogic = 2'b10;
	    end	  
	  WriteData:
	    begin
	       NEXT_STATE <= Idle;
	       OutputLogic = 2'b10;
	    end
	  default:
	    begin
	       NEXT_STATE <= Idle;
	       OutputLogic = 2'b00;
	    end
	endcase // case (CURRENT_STATE)
     end // always @ (CURRENT_STATE or MStrobe or Read or Write or Trigger)   

endmodule /* Control */

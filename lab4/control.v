module CacheControl (Strobe, DRW, M, V, 
		     DReady, W, MStrobe, MRW, 
		     RSel, WSel, 
		     clk, reset);
   
   input Strobe;
   input DRW;
   input M;
   input V;
   input clk;
   input reset;   

   output DReady;
   output W;
   output MStrobe;
   output MRW;
   output RSel;
   output WSel;

   wire [7:0] WSCLoadVal;   
   wire       CtrSig;
   wire       ReadyEn;
   wire       LdCtr;
   wire       Ready;   
   reg [7:0]  OutputLogic;   

   assign WSCLoadVal = 8'd100;   
   assign DReady = Strobe & ~((ReadyEn & M & V & ~DRW) || Ready);
   assign {LdCtr, ReadyEn, Ready, W, MStrobe, MRW, RSel, WSel} = OutputLogic;

   parameter [3:0] 
     Idle      = 4'b0000,
     Read      = 4'b0001,
     ReadMiss  = 4'b0010,
     ReadMem   = 4'b0011,
     ReadData  = 4'b0100,
     Write     = 4'b0101,
     WriteHit  = 4'b0110,
     WriteMiss = 4'b0111,
     WriteMem  = 4'b1000,
     WriteData = 4'b1001;

   reg [3:0] 	   CURRENT_STATE;
   reg [3:0] 	   NEXT_STATE;

   // counter
   wait_state WaitStateCtr (LdCtr, WSCLoadVal, CtrSig, clk);

   
	initial CURRENT_STATE = Idle;
   //Update FSM as negative edge, so new data is ready before the next PC clock cycle to prevent (win) races
   always @(negedge clk or reset) 
      begin
			if(reset) CURRENT_STATE = Idle;
      else CURRENT_STATE = NEXT_STATE;
   end
   
	//Only trigger on posedge of some variables, because on a read hit these change 
	//to 0 before we've gone to IDLE.
	//The CPU just has to be sure to lower Strobe after an access before negedge clk, which shouldn't be hard.
   always @(CURRENT_STATE or posedge Strobe or DRW or posedge M or posedge V or CtrSig)
   begin
      case(CURRENT_STATE)
         Idle:
         begin
            OutputLogic = {1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0};
            if(!Strobe) NEXT_STATE = Idle;
            else if(DRW) NEXT_STATE = Write;
            else NEXT_STATE = Read;
         end
         
         Read:
         begin
            OutputLogic = {1'b1,1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0};
            if(M&V) NEXT_STATE = Idle;
            else NEXT_STATE = ReadMiss;
         end
         ReadMiss:
         begin
            OutputLogic = {1'b1,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0};
            NEXT_STATE = ReadMem;
         end
         ReadMem:
         begin
            OutputLogic = {1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,1'b0};
            if(CtrSig) NEXT_STATE = ReadData;
            else NEXT_STATE = ReadMem;
         end
         ReadData:
         begin
            OutputLogic = {1'b0,1'b0,1'b1,1'b1,1'b0,1'b1,1'b1,1'b0};
            NEXT_STATE = Idle;
         end
         
         
         Write:
			begin
				OutputLogic = {1'b1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0};
				if(M&V) NEXT_STATE = WriteHit;
				else NEXT_STATE = WriteMiss;
			end
			WriteMiss:
			begin
				OutputLogic = {1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0};
				NEXT_STATE = WriteMem;
			end
         WriteHit:
			begin
				OutputLogic = {1'b1,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0};
				NEXT_STATE = WriteMem;
			end
			WriteMem:
			begin
				OutputLogic = {1'b0,1'b0,1'b0,1'b0,1'b1,1'b1,1'b0,1'b0};
				if(CtrSig) NEXT_STATE = WriteData;
				else NEXT_STATE = WriteMem;
			end
			WriteData:
			begin
				OutputLogic = {1'b0,1'b0,1'b1,1'b1,1'b0,1'b1,1'b0,1'b1};
				NEXT_STATE = Idle;
			end
         
			
      endcase
   end

endmodule // CacheControl




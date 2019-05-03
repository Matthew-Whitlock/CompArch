//------------------------------------------------
// cache.v
// James E. Stine
// Oklahoma State University
// ECEN 4243
// Direct-mapped cache
//------------------------------------------------

module cache (PStrobe, PRW, PAddress, PDataIn, PReady, PDataOut,
              MStrobe, MRW, MAddress, MDataOut, MDataIn, 
	      Reset, clk);

   input           PStrobe;             // Strobe
   input           PRW;                 // R/W
   input [31:0]    PAddress;            // Address
   input [31:0]    PDataIn;             // DataIn
   output          PReady;              // Ready
   output [31:0]   PDataOut;            // DataOut

   output          MStrobe;             // MMem Strobe
   output          MRW;                 // MMem R/W
   output [31:0]   MAddress;            // MMem Address
   input [31:0]    MDataOut;            // MMem DataOut
   output [31:0]   MDataIn;             // MMem DataIn   
   
   input           Reset;
   input           clk;

   wire [31:0] 	   DDataOut;   
   wire [17:0] 	   TagRamTag;
   wire [31:0] 	   DataRamDataIn, DataRamDataOut;

   // Output Address/Data for Main Memory
   assign MAddress = PAddress;
   assign MDataIn = PDataIn;   

   // Tag RAM
   tag_ram TagRam (TagRamTag, Write, ~clk, PAddress[13:2], 
		  PAddress[31:14]);
   // Valid (V) RAM
   valid_ram ValidRam (Valid, Write, ~clk, PAddress[13:2], 1'b1);
   mux2 #(32) m1 (DataRamDataOut, MDataOut, RSel, PDataOut);   
   mux2 #(32) m2 (PDataIn, MDataOut, WSel, DDataOut);
   // Data RAM for direct-mapped cache (4KB)
   data_ram DataRam (DataRamDataOut, Write, ~clk, PAddress[13:2], DDataOut);
   // check cache for Match
   Comparator Comparator (PAddress[31:14], TagRamTag, Match);
   // brains of cache (FSM)
   CacheControl CacheControl (PStrobe, PRW, Match, Valid,
			      PReady, Write, MStrobe, MRW,
			      RSel, WSel, clk, Reset);
   

endmodule // Cache


module top (input  logic        clk, reset, 
            output logic [31:0] WriteDataM, DataAdrM, 
            output logic        MemWriteM);
   
   logic [31:0] 		PCF, InstrF, ReadDataM, MAddress, MDataOut, MDataIn;
   
   // instantiate processor and memories
   arm arm (clk, reset, PCF, InstrF, MemWriteM, DataAdrM, 
            WriteDataM, ReadDataM, PStrobe, PReady);
   // cache
   cache cache (PStrobe, MemWriteM, DataAdrM, WriteDataM, PReady, ReadDataM,
                MStrobe, MRW, MAddress, MDataOut, MDataIn,
					 Reset, clk);
   // main memories   
   imem imem(PCF, InstrF);
	
   dmem_wait dmem(MDataOut, MRW, ~clk, MAddress, MDataIn, MStrobe);
   
endmodule // top

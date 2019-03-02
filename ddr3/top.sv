module top (input  logic        clk, reset, 
            output logic [31:0] WriteDataM, DataAdrM, 
            output logic        MemWriteM);
   
   logic [31:0] 		PCF, InstrF, ReadDataM;
   logic 			MStrobe;
   logic [199:0] 		testvectors[100:0];
   
   // instantiate processor and memories
   arm arm (clk, reset, WriteData, DataAdr, MemWrite, MStrobe);
   // cache
   MemControl dmem (Done, WriteData, MStrobe, MemWrite, Trigger, clk, reset);

   // main memories
   imem imem (PC, Instr);
   dmem dmem (Trigger, ReadData, MemWrite, clk, DataAdr, WriteData, MStrobe)
     
   
endmodule // top

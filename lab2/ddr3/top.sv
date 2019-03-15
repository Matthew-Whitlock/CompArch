module top (input  logic        clk, reset, 
            output logic [31:0] WriteDataM, DataAdrM, 
            output logic        MemWriteM);
   
   logic [31:0] 		PC, Instr, ReadData, WriteData, DataAdr;
   logic 				MStrobe, PReady;
   logic [199:0] 		testvectors[100:0];
   
   // instantiate processor and memories
   arm arm (clk, reset, PC, Instr, MemWrite, DataAdr, WriteData, ReadData, MStrobe, PReady);
   // cache
   MemControl dmem_ctrl (PReady, MStrobe, MemWrite, clk, reset);

   // main memories
   imem imem (PC, Instr);
   dmem_wait dmem (ReadData, MemWrite, clk, DataAdr, WriteData, MStrobe);
   
endmodule // top

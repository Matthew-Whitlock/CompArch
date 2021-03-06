//------------------------------------------------
// dmem_big.v
// James E. Stine
// February 1, 2018
// Oklahoma State University
// ECEN 4243
// Harvard Architecture Data Memory (Big Endian)
//------------------------------------------------

module dmem (mem_out, r_w, clk, mem_addr, mem_data);

   output [31:0] mem_out;
   input 	 r_w;
   input 	 clk;   
   input [31:0]  mem_addr;
   input [31:0]  mem_data;

   // Choose smaller memory to speed simulation
   //   through smaller AddrSize (only used to
   //   allocate memory size -- processor sees
   //   32-bits)
   parameter AddrSize = 16;
   parameter WordSize = 8;

   reg [WordSize-1:0] RAM[((1<<AddrSize)-1):0];   

   // Read memory
   //   byte addressed, but appears as 32b to processor
   assign mem_out = {RAM[mem_addr], RAM[mem_addr+1],
                     RAM[mem_addr+2], RAM[mem_addr+3]};

   // Write memory
   always @(posedge clk) 
   begin
     if (r_w)
       {RAM[mem_addr], RAM[mem_addr+1], 
          RAM[mem_addr+2], RAM[mem_addr+3]} <= mem_data; 
   end

   
endmodule // mem


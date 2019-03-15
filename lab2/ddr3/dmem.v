//------------------------------------------------
// mem.v
// James E. Stine/Alex Underwood
// April 1, 2018 (no joke)
// Oklahoma State University
// ECEN 4243
// Princeton Architecture Data Memory (Big Endian)
//   with wait states (100 cycles)
//------------------------------------------------

module dmem_wait (mem_out, r_w, clk, mem_addr, mem_data, mstrobe);

   output [31:0] mem_out;
   input 	 r_w;
   input 	 clk;   
   input [31:0]  mem_addr;
   input [31:0]  mem_data;
   input 	 mstrobe;
   reg [31:0] 	 mem_out;   

   // Choose smaller memory to speed simulation
   //   through smaller AddrSize (only used to
   //   allocate memory size -- processor sees
   //   32-bits)
   parameter AddrSize = 16;
   parameter WordSize = 8;

   reg [WordSize-1:0] RAM[((1<<AddrSize)-1):0];   

   // Read/Write memory synchronous
   always @(posedge clk) begin
      if (mstrobe == 1'b1) begin
	 if (r_w)
	   {RAM[mem_addr], RAM[mem_addr+1], 
	    RAM[mem_addr+2], RAM[mem_addr+3]} <= mem_data;
	 // Zzzzzz for 16 cycles
	  repeat (16)
	    @(posedge clk);	 
	 //   byte addressed, but appears as 32b to processor
	 mem_out <= {RAM[mem_addr], RAM[mem_addr+1],
		     RAM[mem_addr+2], RAM[mem_addr+3]};		     
      end
   end
   
endmodule // dmem_wait



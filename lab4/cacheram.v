//------------------------------------------------
// cacheram.v
// James E. Stine
// April 1, 2018 (no joke)
// Oklahoma State University
// ECEN 4243
// Cache RAM (not byte addressed to simplify)
//------------------------------------------------

module tag_ram (mem_out, r_w, clk, mem_addr, mem_data);

   input [11:0]    mem_addr;   
   input 	   r_w;
   input 	   clk;   
   input [17:0]    mem_data;
   output [17:0]   mem_out;   

   // Choose smaller memory to speed simulation
   //   through smaller AddrSize (only used to
   //   allocate memory size 
   parameter AddrSize = 12;
   parameter WordSize = 18;

   reg [WordSize-1:0] RAM[((1<<AddrSize)-1):0];   

   assign mem_out = RAM[mem_addr];
   
   // Write memory
   always @(posedge clk) 
     begin
	if (r_w)
	  RAM[mem_addr] <= mem_data;
     end	
   
endmodule // tag_ram

module valid_ram (mem_out, r_w, clk, mem_addr, mem_data);

   input [11:0]    mem_addr;   
   input 	     r_w;
   input 	     clk;
   input 	     mem_data;
   output 	     mem_out;   

   // Choose smaller memory to speed simulation
   //   through smaller AddrSize (only used to
   //   allocate memory size 
   parameter AddrSize = 12;

   reg		   RAM[((1<<AddrSize)-1):0];

   assign mem_out = RAM[mem_addr];   

   // Write memory
   always @(posedge clk) 
     begin
	if (r_w)
	  RAM[mem_addr] <= mem_data;
     end   

endmodule // valid_ram

module data_ram (mem_out, r_w, clk, mem_addr, mem_data);

   input [11:0]    mem_addr;   
   input 	     r_w;
   input 	     clk;
   input [31:0]      mem_data;
   output [31:0]     mem_out;   
   
   // Choose smaller memory to speed simulation
   //   through smaller AddrSize (only used to
   //   allocate memory size 
   parameter AddrSize = 12;
   parameter WordSize = 32;

   reg [WordSize-1:0] RAM[((1<<AddrSize)-1):0];

   assign mem_out = RAM[mem_addr];
   
   // Write memory
   always @(posedge clk) 
     begin
	if (r_w)
	  RAM[mem_addr] <= mem_data;
     end   

endmodule // data_ram





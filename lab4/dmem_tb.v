//
// File name : tb.v
// Title     : stimulus
// project   : ECEN3233
// Library   : test
// Author(s) : James E. Stine, Jr.
// Purpose   : definition of modules for testbench 
// notes :   
//
// Copyright Oklahoma State University
//

// Top level stimulus module

module stimulus;

   reg clk;  // Always declared so can simulate based on clock

   // Declare variables for stimulating input
   wire [31:0] mem_out;
   reg 	       r_w;
   reg [31:0]  mem_addr;
   reg [31:0]  mem_data;
   reg 	       mstrobe;
   
   integer     handle3;
   integer     desc3;   
   
   // Instantiate the design block counter
   dmem_wait dut (mem_out, r_w, clk, mem_addr, mem_data, mstrobe);   

   // Setup the clock to toggle every 1 time units 
   initial 
     begin	
	clk = 1'b1;
	forever #5 clk = ~clk;
     end

   initial
     begin
	// Gives output file name
	handle3 = $fopen("dmem.out");
	// Tells when to finish simulation
	#1500 $finish;		
     end

   always 
     begin
	desc3 = handle3;
	#5 $fdisplay(desc3, "%b %b %h %b %h || %h", 
		     clk, mstrobe, mem_addr, r_w, mem_data, mem_out);
     end

   // Stimulate the Input Signals
   initial
     begin
	// Add your test vectors here
	#0  r_w = 1'b0;	
	#0  mstrobe = 1'b0;
	#0  mem_data = 32'h302f_caaa;	
	#43 mem_addr = 32'h20;
	#58 mstrobe = 1'b1;
	#0  r_w = 1'b1;		
	#10 mstrobe = 1'b0;	
	#90 r_w = 1'b0;
	#0  mem_addr = 32'h0;
	#40 mem_addr = 32'h24;
	#0  mstrobe = 1'b1;
	#10 mstrobe = 1'b0;	
     end

endmodule // stimulus






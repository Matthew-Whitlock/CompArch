module tb ();

   logic [31:0] a, b;   
   logic [1:0] 	ALUControl;   
   logic [31:0] Result;   
   logic [3:0] 	ALUFlags;   
   
   logic clk;

   integer     handle3;
   integer     desc3;      

   // Instantiate Device Under Test
   alu dut (a, b, ALUControl, Result, ALUFlags);   

   // Setup the clock to toggle every 1 time units 
   initial 
     begin	
	clk = 1'b1;
	forever #5 clk = ~clk;
     end

   initial
     begin
	handle3 = $fopen("alu.out");
	#100 $finish;		
     end

   always 
     begin
	desc3 = handle3;
	#5 $fdisplay(desc3, "%b %h %h || %h %b", 
		     ALUControl, a, b, Result, ALUFlags);
     end

   // Stimulate the Input Signals
   initial
     begin
	// Add your test vectors here
	#0  a = 32'h10;
	#0  b = 32'h8;
	#0  ALUControl = 2'b01;
     end

endmodule // tb

module alu (input  logic [31:0] a, b,
            input  logic [1:0]  ALUControl,
            output logic [31:0] Result,
            output logic [3:0]  ALUFlags);
   
   logic 			neg, zero, carry, overflow;
   logic [31:0] 		condinvb;
   logic [32:0] 		sum;
   
   assign condinvb = ALUControl[0] ? ~b : b;
   assign sum = a + condinvb + ALUControl[0];

   always_comb
     casex (ALUControl[1:0])
       2'b0?: Result = sum;
       2'b10: Result = a & b;
       2'b11: Result = a | b;
     endcase

   assign neg      = Result[31];
   assign zero     = (Result == 32'b0);
   assign carry    = (ALUControl[1] == 1'b0) & sum[32];
   assign overflow = (ALUControl[1] == 1'b0) & 
                     ~(a[31] ^ b[31] ^ ALUControl[0]) & 
                     (a[31] ^ sum[31]); 
   assign ALUFlags    = {neg, zero, carry, overflow};
   
endmodule // alu
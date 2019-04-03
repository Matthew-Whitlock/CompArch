module alu (input  logic [31:0] a, b,
            input  logic [3:0]  ALUControl,
			input  logic		carry_in,
			output logic [31:0] Result,
            output logic [3:0]  ALUFlags);
   
   logic 			neg, zero, carry, overflow;
   logic [31:0] 		condinvb;
   logic [32:0] 		sum;
   logic [31:0]     carrycompensator;
   
   always_comb
     casex ({ALUControl[2], ALUControl[0], carry_in})
        3'b0??: carrycompensator = 32'd0; //ALUControl[2] specifies ADC or SBC.
        3'b100: carrycompensator = 32'd0; //Adding, carry is 0
        3'b101: carrycompensator = 32'd1; //Adding, carry is 1
        3'b110: carrycompensator = -32'd1;//Subtracting, carry is 0 
        3'b111: carrycompensator = 32'd0; //Subtracting, carry is 0 
     endcase
   
   assign condinvb = ALUControl[0] ? ~b : b;
   assign sum = a + condinvb + ALUControl[0] + carrycompensator;
   

   always_comb
     casex (ALUControl[3:0])
       4'b0?0?: Result = sum;
       4'b0010: Result = a & b;
	   4'b0110: Result = a & ~b;
       4'b0011: Result = a | b;
	   4'b0111: Result = a ^ b;
	   4'b1000: Result = ~b;
     endcase
   
   assign neg      = Result[31];
   assign zero     = (Result == 32'b0);
   assign carry    = (ALUControl[1] == 1'b0) & sum[32];
   assign overflow = (ALUControl[1] == 1'b0) & 
                     ~(a[31] ^ b[31] ^ ALUControl[0]) & 
                     (a[31] ^ sum[31]); 
   assign ALUFlags    = {neg, zero, carry, overflow};
   
endmodule // alu
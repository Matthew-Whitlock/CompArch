module wait_state (Load, LoadValue, Sig, clk);
   
   input           Load;
   input [7:0] 	   LoadValue;
   output          Sig;
   input           clk;

   reg [7:0] 	   count;

   // Assign that we are done
   assign Sig = count == 8'h0;
   // Decrement counter on clk edge
   always @(posedge clk) 
     begin
	if (Load)
	  count = LoadValue;
	else
	  count = count - 8'b1;
     end
   
endmodule // wait_state

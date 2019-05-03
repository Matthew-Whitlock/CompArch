module Comparator (input [17:0] Tag1, input [17:0] Tag2, output Match);

   assign Match = (Tag1 == Tag2);

endmodule // Comparator

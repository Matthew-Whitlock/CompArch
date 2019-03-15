This is the baseline HDL files for the pipelined ARMv8 ARM.

Replace dmem_big.v or dmem_litte.v with dmem.v for correct endianness.
This will depend on how your assembly is compiled (see arm2hex/arm3hex).

The DO file writes memory contents to imemory.dat and demory.dat for
instruction and data memory, respectively.  Check these files to see
if you use a memory instruction to see if its successful.

The ALU.sv file is the ALU with a sample testbench you can use to test
a design you create  (e.g., shifter).  It allows you to have a file
you can use to build any logic and test it.  The testbench is included
in the alu.sv out of convenience -- it can easily be in a separate
file.  To run, type "vsim -do alu.do -c" and the output (based on the
testbench) should print to a file alu.out


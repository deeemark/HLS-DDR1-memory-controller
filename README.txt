DDR Controller in S2CBench Layout
================================

This project follows the same basic structure as the  DARClab S2CBench:

  ddr_controller.cpp
  ddr_controller.h
  tb_ddr_controller.cpp
  tb_ddr_controller.h
  define.h
  main.cpp
  ddr_in_data.txt
  ddr_output_golden.txt
  ddr_output.txt
  ddrdiff.txt
  Makefile / Makefile.GNU

The testbench has separate send(), recv(), and compare_results() functions.

Input file format:
  op address data_hi data_lo mask

op=1 is WRITE.
op=0 is READ.

The 64-bit value is transferred across the controller-to-PHY
interface as two 32-bit words:
    Word 0:
        572657937 decimal = 0x22221111
    Word 1:
        1145320243 decimal = 0x44443333

The supplied test performs one BL4 x16 write and one read. For the READ,
data_hi/data_lo also drive the behavioral PHY return data.

Build:
  make -f Makefile.GNU clean
  make -f Makefile.GNU
  ./ddr_controller.exe

Expected ending:
  Finished simulation SUCCESSFULLY

Wave:
  make -f Makefile.GNU clean
  make -f Makefile.GNU wave
  ./ddr_controller.exe

Generated waveform:
  trace_behav.vcd

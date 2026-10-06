#ifndef TB_DDR_CONTROLLER_H_
#define TB_DDR_CONTROLLER_H_

#include "define.h"
#include <systemc.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

SC_MODULE(test_DDR_CONTROLLER)
{
    sc_in<bool> clk;
    sc_in<bool> rst;

    sc_in<bool>          req_ready;
    sc_in<bool>          rsp_valid;
    sc_in<sc_uint<64> >  rsp_rdata;
    sc_in<bool>          init_done;

    sc_in<bool>          ddr_cke;
    sc_in<bool>          ddr_cs_n;
    sc_in<bool>          ddr_ras_n;
    sc_in<bool>          ddr_cas_n;
    sc_in<bool>          ddr_we_n;
    sc_in<sc_uint<2> >   ddr_ba;
    sc_in<sc_uint<14> >  ddr_addr;

    sc_in<bool>          phy_wr_valid;
    sc_in<sc_uint<32> >  phy_wr_data;
    sc_in<sc_uint<4> >   phy_wr_mask;

    sc_out<bool>          req_valid;
    sc_out<bool>          req_write;
    sc_out<sc_uint<24> >  req_addr;
    sc_out<sc_uint<64> >  req_wdata;
    sc_out<sc_uint<8> >   req_wmask;

    /* DDR memory-model verification status */
    sc_in<sc_uint<16> >   ddr_timing_error_count;
    sc_in<sc_uint<8> >    ddr_last_error_code;

    FILE *in_ddr_file;
    FILE *out_ddr_file;
    FILE *out_ddr_file_read;
    FILE *out_ddr_golden_file;
    FILE *diff_file;

    bool test_done;

    void compare_results();
    void recv();
    void send();

    SC_CTOR(test_DDR_CONTROLLER)
    {
        test_done = false;

        SC_CTHREAD(send, clk.pos());
        reset_signal_is(rst, false);

        SC_CTHREAD(recv, clk.pos());
        reset_signal_is(rst, false);
    }

    ~test_DDR_CONTROLLER() {}
};

#endif

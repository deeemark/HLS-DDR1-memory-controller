// g++ -o ddr_controller.exe main.cpp tb_ddr_controller.cpp ddr_controller.cpp ddr_memory_model.cpp -I$SYSTEMC_HOME/include -L$SYSTEMC_HOME/lib-linux64 -lsystemc -lm
#include "ddr_controller.h"
#include "tb_ddr_controller.h"
#include "ddr_memory_model.h"

int sc_main(int argc, char **argv)
{
    sc_clock clk("clk", 6, SC_NS, 0.5, 3, SC_NS, true);

    sc_signal<bool> rst;

    sc_signal<bool>          req_valid;
    sc_signal<bool>          req_write;
    sc_signal<sc_uint<24> >  req_addr;
    sc_signal<sc_uint<64> >  req_wdata;
    sc_signal<sc_uint<8> >   req_wmask;

    sc_signal<bool>          req_ready;
    sc_signal<bool>          rsp_valid;
    sc_signal<sc_uint<64> >  rsp_rdata;

    sc_signal<bool>          ddr_cke;
    sc_signal<bool>          ddr_cs_n;
    sc_signal<bool>          ddr_ras_n;
    sc_signal<bool>          ddr_cas_n;
    sc_signal<bool>          ddr_we_n;
    sc_signal<sc_uint<2> >   ddr_ba;
    sc_signal<sc_uint<14> >  ddr_addr;

    sc_signal<bool>          phy_wr_valid;
    sc_signal<sc_uint<32> >  phy_wr_data;
    sc_signal<sc_uint<4> >   phy_wr_mask;

    sc_signal<bool>          phy_rd_valid;
    sc_signal<sc_uint<32> >  phy_rd_data;

    sc_signal<bool>          init_done;

    /* Behavioral DDR memory-model verification status */
    sc_signal<bool>          ddr_timing_error;
    sc_signal<sc_uint<16> >  ddr_timing_error_count;
    sc_signal<sc_uint<8> >   ddr_last_error_code;
    sc_signal<sc_uint<4> >   ddr_bank_open_mask;
    sc_signal<sc_uint<4> >   ddr_auto_precharge_mask;

    ddr_controller u_DDR_CONTROLLER("DDR_CONTROLLER");
    ddr_memory_model u_DDR_MEMORY("DDR_MEMORY");
    test_DDR_CONTROLLER test("test_DDR_CONTROLLER");

    u_DDR_CONTROLLER.clk(clk);
    u_DDR_CONTROLLER.rst_n(rst);

    u_DDR_CONTROLLER.req_valid(req_valid);
    u_DDR_CONTROLLER.req_write(req_write);
    u_DDR_CONTROLLER.req_addr(req_addr);
    u_DDR_CONTROLLER.req_wdata(req_wdata);
    u_DDR_CONTROLLER.req_wmask(req_wmask);

    u_DDR_CONTROLLER.req_ready(req_ready);
    u_DDR_CONTROLLER.rsp_valid(rsp_valid);
    u_DDR_CONTROLLER.rsp_rdata(rsp_rdata);

    u_DDR_CONTROLLER.ddr_cke(ddr_cke);
    u_DDR_CONTROLLER.ddr_cs_n(ddr_cs_n);
    u_DDR_CONTROLLER.ddr_ras_n(ddr_ras_n);
    u_DDR_CONTROLLER.ddr_cas_n(ddr_cas_n);
    u_DDR_CONTROLLER.ddr_we_n(ddr_we_n);
    u_DDR_CONTROLLER.ddr_ba(ddr_ba);
    u_DDR_CONTROLLER.ddr_addr(ddr_addr);

    u_DDR_CONTROLLER.phy_wr_valid(phy_wr_valid);
    u_DDR_CONTROLLER.phy_wr_data(phy_wr_data);
    u_DDR_CONTROLLER.phy_wr_mask(phy_wr_mask);

    u_DDR_CONTROLLER.phy_rd_valid(phy_rd_valid);
    u_DDR_CONTROLLER.phy_rd_data(phy_rd_data);

    u_DDR_CONTROLLER.init_done(init_done);

    /*
     * Behavioral DDR memory model. The controller still owns all command
     * and write-side signals. The memory model now owns phy_rd_* and
     * returns data according to its backing store.
     */
    u_DDR_MEMORY.clk(clk);
    u_DDR_MEMORY.rst_n(rst);

    u_DDR_MEMORY.ddr_cke(ddr_cke);
    u_DDR_MEMORY.ddr_cs_n(ddr_cs_n);
    u_DDR_MEMORY.ddr_ras_n(ddr_ras_n);
    u_DDR_MEMORY.ddr_cas_n(ddr_cas_n);
    u_DDR_MEMORY.ddr_we_n(ddr_we_n);
    u_DDR_MEMORY.ddr_ba(ddr_ba);
    u_DDR_MEMORY.ddr_addr(ddr_addr);

    u_DDR_MEMORY.phy_wr_valid(phy_wr_valid);
    u_DDR_MEMORY.phy_wr_data(phy_wr_data);
    u_DDR_MEMORY.phy_wr_mask(phy_wr_mask);

    u_DDR_MEMORY.phy_rd_valid(phy_rd_valid);
    u_DDR_MEMORY.phy_rd_data(phy_rd_data);

    u_DDR_MEMORY.timing_error(ddr_timing_error);
    u_DDR_MEMORY.timing_error_count(ddr_timing_error_count);
    u_DDR_MEMORY.last_error_code(ddr_last_error_code);
    u_DDR_MEMORY.bank_open_mask(ddr_bank_open_mask);
    u_DDR_MEMORY.auto_precharge_mask(ddr_auto_precharge_mask);

    test.clk(clk);
    test.rst(rst);

    test.req_ready(req_ready);
    test.rsp_valid(rsp_valid);
    test.rsp_rdata(rsp_rdata);
    test.init_done(init_done);

    test.ddr_cke(ddr_cke);
    test.ddr_cs_n(ddr_cs_n);
    test.ddr_ras_n(ddr_ras_n);
    test.ddr_cas_n(ddr_cas_n);
    test.ddr_we_n(ddr_we_n);
    test.ddr_ba(ddr_ba);
    test.ddr_addr(ddr_addr);

    test.phy_wr_valid(phy_wr_valid);
    test.phy_wr_data(phy_wr_data);
    test.phy_wr_mask(phy_wr_mask);

    test.req_valid(req_valid);
    test.req_write(req_write);
    test.req_addr(req_addr);
    test.req_wdata(req_wdata);
    test.req_wmask(req_wmask);

    test.ddr_timing_error_count(ddr_timing_error_count);
    test.ddr_last_error_code(ddr_last_error_code);

#ifdef WAVE_DUMP
    sc_trace_file *trace_file =
        sc_create_vcd_trace_file("trace_behav");

    sc_trace(trace_file, clk,          "clk");
    sc_trace(trace_file, rst,          "rst");
    sc_trace(trace_file, init_done,    "init_done");

    sc_trace(trace_file, req_valid,    "req_valid");
    sc_trace(trace_file, req_ready,    "req_ready");
    sc_trace(trace_file, req_write,    "req_write");
    sc_trace(trace_file, req_addr,     "req_addr");
    sc_trace(trace_file, req_wdata,    "req_wdata");
    sc_trace(trace_file, req_wmask,    "req_wmask");

    sc_trace(trace_file, rsp_valid,    "rsp_valid");
    sc_trace(trace_file, rsp_rdata,    "rsp_rdata");

    sc_trace(trace_file, ddr_cke,      "ddr_cke");
    sc_trace(trace_file, ddr_cs_n,     "ddr_cs_n");
    sc_trace(trace_file, ddr_ras_n,    "ddr_ras_n");
    sc_trace(trace_file, ddr_cas_n,    "ddr_cas_n");
    sc_trace(trace_file, ddr_we_n,     "ddr_we_n");
    sc_trace(trace_file, ddr_ba,       "ddr_ba");
    sc_trace(trace_file, ddr_addr,     "ddr_addr");

    sc_trace(trace_file, phy_wr_valid, "phy_wr_valid");
    sc_trace(trace_file, phy_wr_data,  "phy_wr_data");
    sc_trace(trace_file, phy_wr_mask,  "phy_wr_mask");
    sc_trace(trace_file, phy_rd_valid, "phy_rd_valid");
    sc_trace(trace_file, phy_rd_data,  "phy_rd_data");

    sc_trace(trace_file, ddr_timing_error,       "ddr_timing_error");
    sc_trace(trace_file, ddr_timing_error_count, "ddr_timing_error_count");
    sc_trace(trace_file, ddr_last_error_code,    "ddr_last_error_code");
    sc_trace(trace_file, ddr_bank_open_mask,     "ddr_bank_open_mask");
    sc_trace(trace_file, ddr_auto_precharge_mask,"ddr_auto_precharge_mask");
#endif

    sc_start(6, SC_NS);
    rst.write(false);

    sc_start(12, SC_NS);
    rst.write(true);

    sc_start();

#ifdef WAVE_DUMP
    sc_close_vcd_trace_file(trace_file);
#endif

    return 0;
}

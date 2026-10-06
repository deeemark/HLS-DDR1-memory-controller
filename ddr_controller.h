#ifndef DDR_CONTROLLER_H
#define DDR_CONTROLLER_H

#include <systemc.h>

/*
 * SystemC DDR1 controller
 *
 * Target:
 *   - JEDEC JESD79F DDR SDRAM
 *   - 256 Mb x16
 *   - 4 banks
 *   - DDR333-class controller clock (6 ns nominal)
 *   - Burst length = 4
 *   - Sequential burst
 *   - CAS latency = 2.5
 *
 * This is the command/controller layer. The physical DDR I/O
 * (CK/CK#, DQS edge timing, SSTL_2, DDR input/output registers)
 * is separated behind a simple PHY-side interface.
 */

SC_MODULE(ddr_controller)
{
    /* Clock / reset */
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    /* User request interface */
    sc_in<bool>          req_valid;
    sc_in<bool>          req_write;
    sc_in<sc_uint<24> >  req_addr;
    sc_in<sc_uint<64> >  req_wdata;
    sc_in<sc_uint<8> >   req_wmask;

    sc_out<bool>          req_ready;
    sc_out<bool>          rsp_valid;
    sc_out<sc_uint<64> >  rsp_rdata;

    /* DDR command/address interface */
    sc_out<bool>          ddr_cke;
    sc_out<bool>          ddr_cs_n;
    sc_out<bool>          ddr_ras_n;
    sc_out<bool>          ddr_cas_n;
    sc_out<bool>          ddr_we_n;
    sc_out<sc_uint<2> >   ddr_ba;
    sc_out<sc_uint<14> >  ddr_addr;

    /*
     * PHY-side interface.
     * One 32-bit PHY word = two x16 DDR data transfers in one CK.
     */
    sc_out<bool>          phy_wr_valid;
    sc_out<sc_uint<32> >  phy_wr_data;
    sc_out<sc_uint<4> >   phy_wr_mask;

    sc_in<bool>           phy_rd_valid;
    sc_in<sc_uint<32> >   phy_rd_data;

    sc_out<bool>          init_done;

    enum {
        T_POWERUP = 33334, /* >= 200 us / 6 ns */
        T_MRD     = 2,
        T_RCD     = 4,     /* ceil(20 ns / 6 ns) */
        T_RP      = 4,     /* ceil(20 ns / 6 ns) */
        T_RAS     = 8,     /* ceil(45 ns / 6 ns) */
        T_RC      = 11,    /* ceil(65 ns / 6 ns) */
        T_RFC     = 13,    /* ceil(75 ns / 6 ns) */
        T_RRD     = 3,     /* ceil(15 ns / 6 ns) */
        T_WR      = 3,     /* ceil(15 ns / 6 ns) */
        T_DLL     = 200,
        T_REFI    = 1300   /* 7.8 us / 6 ns */
    };

    enum state_t {
        S_PWRUP = 0,
        S_INIT_NOP,
        S_INIT_PRE,
        S_INIT_PRE_WAIT,
        S_INIT_EMRS,
        S_INIT_EMRS_WAIT,
        S_INIT_MRS_DLLRST,
        S_INIT_MRS_WAIT,
        S_INIT_DLL_WAIT,
        S_INIT_PRE2,
        S_INIT_PRE2_WAIT,
        S_INIT_REF1,
        S_INIT_REF1_WAIT,
        S_INIT_REF2,
        S_INIT_REF2_WAIT,
        S_INIT_MRS_NORMAL,
        S_INIT_FINAL_WAIT,

        S_IDLE,
        S_ACT,
        S_TRCD_WAIT,
        S_READ_CMD,
        S_READ_DATA,
        S_READ_RECOVER,
        S_WRITE_CMD,
        S_WRITE_DATA0,
        S_WRITE_DATA1,
        S_WRITE_RECOVER,

        S_REFRESH_PRE,
        S_REFRESH_PRE_WAIT,
        S_REFRESH_CMD,
        S_REFRESH_WAIT
    };

    state_t state;

    sc_uint<32> wait_count;
    sc_uint<16> refresh_count;

    bool initialized;
    bool lat_write;

    sc_uint<24> lat_addr;
    sc_uint<64> lat_wdata;
    sc_uint<8>  lat_wmask;

    sc_uint<13> lat_row;
    sc_uint<2>  lat_bank;
    sc_uint<9>  lat_col;

    sc_uint<64> read_buf;
    sc_uint<2>  read_words;

    /* Hardware process */
    void main_thread();

    /* Combinational helper routines called only by main_thread */
    void drive_nop();
    void drive_precharge_all();
    void drive_refresh();
    void drive_emrs();
    void drive_mrs(bool dll_reset);
    void drive_active();
    void drive_read_ap();
    void drive_write_ap();
    void decode_address();

    SC_CTOR(ddr_controller)
    {
        SC_CTHREAD(main_thread, clk.pos());
        reset_signal_is(rst_n, false);
    }
};

#endif

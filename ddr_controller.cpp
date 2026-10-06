#include "ddr_controller.h"

void ddr_controller::drive_nop()
{
    ddr_cs_n  = false;
    ddr_ras_n = true;
    ddr_cas_n = true;
    ddr_we_n  = true;
    ddr_ba    = 0;
    ddr_addr  = 0;
}

void ddr_controller::drive_precharge_all()
{
    sc_uint<14> a;

    a = 0;
    a[10] = 1;

    ddr_cs_n  = false;
    ddr_ras_n = false;
    ddr_cas_n = true;
    ddr_we_n  = false;
    ddr_ba    = 0;
    ddr_addr  = a;
}

void ddr_controller::drive_refresh()
{
    ddr_cs_n  = false;
    ddr_ras_n = false;
    ddr_cas_n = false;
    ddr_we_n  = true;
    ddr_ba    = 0;
    ddr_addr  = 0;
}

void ddr_controller::drive_emrs()
{
    /*
     * BA1:BA0 = 01 selects EMRS.
     * A0 = 0 : DLL enabled
     * A1 = 0 : normal drive strength
     */
    ddr_cs_n  = false;
    ddr_ras_n = false;
    ddr_cas_n = false;
    ddr_we_n  = false;
    ddr_ba    = 1;
    ddr_addr  = 0;
}

void ddr_controller::drive_mrs(bool dll_reset)
{
    sc_uint<14> a;

    /*
     * A2:A0 = 010 : BL4
     * A3     = 0   : sequential
     * A6:A4 = 110 : CAS latency 2.5
     * A8     = 1   : DLL reset when requested
     */
    a = 0;
    a[2] = 0;
    a[1] = 1;
    a[0] = 0;

    a[3] = 0;

    a[6] = 1;
    a[5] = 1;
    a[4] = 0;

    if (dll_reset)
        a[8] = 1;
    else
        a[8] = 0;

    ddr_cs_n  = false;
    ddr_ras_n = false;
    ddr_cas_n = false;
    ddr_we_n  = false;
    ddr_ba    = 0;
    ddr_addr  = a;
}

void ddr_controller::drive_active()
{
    sc_uint<14> a;

    a = 0;
    a.range(12,0) = lat_row;

    ddr_cs_n  = false;
    ddr_ras_n = false;
    ddr_cas_n = true;
    ddr_we_n  = true;
    ddr_ba    = lat_bank;
    ddr_addr  = a;
}

void ddr_controller::drive_read_ap()
{
    sc_uint<14> a;

    a = 0;
    a.range(8,0) = lat_col;
    a[10] = 1;  /* auto precharge */

    ddr_cs_n  = false;
    ddr_ras_n = true;
    ddr_cas_n = false;
    ddr_we_n  = true;
    ddr_ba    = lat_bank;
    ddr_addr  = a;
}

void ddr_controller::drive_write_ap()
{
    sc_uint<14> a;

    a = 0;
    a.range(8,0) = lat_col;
    a[10] = 1;  /* auto precharge */

    ddr_cs_n  = false;
    ddr_ras_n = true;
    ddr_cas_n = false;
    ddr_we_n  = false;
    ddr_ba    = lat_bank;
    ddr_addr  = a;
}

void ddr_controller::decode_address()
{
    /*
     * 256 Mb x16 geometry:
     *   row    = 13 bits
     *   bank   = 2 bits
     *   column = 9 bits
     *
     * BL4 requests are aligned to a 4-word boundary.
     */
    lat_col = lat_addr.range(8,0);
    lat_col[1] = 0;
    lat_col[0] = 0;

    lat_bank = lat_addr.range(10,9);
    lat_row  = lat_addr.range(23,11);
}

void ddr_controller::main_thread()
{
    state         = S_PWRUP;
    wait_count    = 0;
    refresh_count = 0;

    initialized   = false;
    lat_write     = false;

    lat_addr      = 0;
    lat_wdata     = 0;
    lat_wmask     = 0;
    lat_row       = 0;
    lat_bank      = 0;
    lat_col       = 0;

    read_buf      = 0;
    read_words    = 0;

    ddr_cke       = false;
    ddr_cs_n      = false;
    ddr_ras_n     = true;
    ddr_cas_n     = true;
    ddr_we_n      = true;
    ddr_ba        = 0;
    ddr_addr      = 0;

    req_ready     = false;
    rsp_valid     = false;
    rsp_rdata     = 0;

    phy_wr_valid  = false;
    phy_wr_data   = 0;
    phy_wr_mask   = 0;

    init_done     = false;

    wait();

    while (true)
    {
        /* Default one-cycle outputs */
        drive_nop();
        req_ready    = false;
        rsp_valid    = false;
        phy_wr_valid = false;
        init_done    = initialized;

        if (initialized)
        {
            if (refresh_count < T_REFI)
                refresh_count = refresh_count + 1;
        }

        switch (state)
        {
        case S_PWRUP:
            ddr_cke = false;

            if (wait_count >= (T_POWERUP - 1))
            {
                wait_count = 0;
                state = S_INIT_NOP;
            }
            else
            {
                wait_count = wait_count + 1;
            }
            break;

        case S_INIT_NOP:
            ddr_cke = true;
            drive_nop();
            state = S_INIT_PRE;
            break;

        case S_INIT_PRE:
            drive_precharge_all();
            wait_count = 0;
            state = S_INIT_PRE_WAIT;
            break;

        case S_INIT_PRE_WAIT:
            if (wait_count >= (T_RP - 1))
            {
                wait_count = 0;
                state = S_INIT_EMRS;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_EMRS:
            drive_emrs();
            wait_count = 0;
            state = S_INIT_EMRS_WAIT;
            break;

        case S_INIT_EMRS_WAIT:
            if (wait_count >= (T_MRD - 1))
            {
                wait_count = 0;
                state = S_INIT_MRS_DLLRST;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_MRS_DLLRST:
            drive_mrs(true);
            wait_count = 0;
            state = S_INIT_MRS_WAIT;
            break;

        case S_INIT_MRS_WAIT:
            if (wait_count >= (T_MRD - 1))
            {
                wait_count = 0;
                state = S_INIT_DLL_WAIT;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_DLL_WAIT:
            if (wait_count >= (T_DLL - 1))
            {
                wait_count = 0;
                state = S_INIT_PRE2;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_PRE2:
            drive_precharge_all();
            wait_count = 0;
            state = S_INIT_PRE2_WAIT;
            break;

        case S_INIT_PRE2_WAIT:
            if (wait_count >= (T_RP - 1))
            {
                wait_count = 0;
                state = S_INIT_REF1;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_REF1:
            drive_refresh();
            wait_count = 0;
            state = S_INIT_REF1_WAIT;
            break;

        case S_INIT_REF1_WAIT:
            if (wait_count >= (T_RFC - 1))
            {
                wait_count = 0;
                state = S_INIT_REF2;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_REF2:
            drive_refresh();
            wait_count = 0;
            state = S_INIT_REF2_WAIT;
            break;

        case S_INIT_REF2_WAIT:
            if (wait_count >= (T_RFC - 1))
            {
                wait_count = 0;
                state = S_INIT_MRS_NORMAL;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_INIT_MRS_NORMAL:
            drive_mrs(false);
            wait_count = 0;
            state = S_INIT_FINAL_WAIT;
            break;

        case S_INIT_FINAL_WAIT:
            if (wait_count >= (T_MRD - 1))
            {
                wait_count = 0;
                refresh_count = 0;
                initialized = true;
                init_done = true;
                state = S_IDLE;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_IDLE:
            req_ready = true;

            if (refresh_count >= T_REFI)
            {
                req_ready = false;
                state = S_REFRESH_PRE;
            }
            else if (req_valid.read())
            {
                lat_write = req_write.read();
                lat_addr  = req_addr.read();
                lat_wdata = req_wdata.read();
                lat_wmask = req_wmask.read();

                decode_address();

                state = S_ACT;
            }
            break;

        case S_ACT:
            drive_active();
            wait_count = 0;
            state = S_TRCD_WAIT;
            break;

        case S_TRCD_WAIT:
            if (wait_count >= (T_RCD - 1))
            {
                wait_count = 0;

                if (lat_write)
                    state = S_WRITE_CMD;
                else
                    state = S_READ_CMD;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_READ_CMD:
            drive_read_ap();
            read_buf = 0;
            read_words = 0;
            state = S_READ_DATA;
            break;

        case S_READ_DATA:
            if (phy_rd_valid.read())
            {
                if (read_words == 0)
                {
                    read_buf.range(31,0) = phy_rd_data.read();
                    read_words = 1;
                }
                else
                {
                    read_buf.range(63,32) = phy_rd_data.read();
                    rsp_rdata = read_buf;
                    rsp_valid = true;
                    wait_count = 0;
                    state = S_READ_RECOVER;
                }
            }
            break;

        case S_READ_RECOVER:
            if (wait_count >= (T_RP - 1))
            {
                wait_count = 0;
                state = S_IDLE;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_WRITE_CMD:
            drive_write_ap();
            state = S_WRITE_DATA0;
            break;

        case S_WRITE_DATA0:
            phy_wr_valid = true;
            phy_wr_data  = lat_wdata.range(31,0);
            phy_wr_mask  = lat_wmask.range(3,0);
            state = S_WRITE_DATA1;
            break;

        case S_WRITE_DATA1:
            phy_wr_valid = true;
            phy_wr_data  = lat_wdata.range(63,32);
            phy_wr_mask  = lat_wmask.range(7,4);
            wait_count = 0;
            state = S_WRITE_RECOVER;
            break;

        case S_WRITE_RECOVER:
            if (wait_count >= ((T_WR + T_RP) - 1))
            {
                wait_count = 0;
                state = S_IDLE;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_REFRESH_PRE:
            drive_precharge_all();
            wait_count = 0;
            state = S_REFRESH_PRE_WAIT;
            break;

        case S_REFRESH_PRE_WAIT:
            if (wait_count >= (T_RP - 1))
            {
                wait_count = 0;
                state = S_REFRESH_CMD;
            }
            else
                wait_count = wait_count + 1;
            break;

        case S_REFRESH_CMD:
            drive_refresh();
            refresh_count = 0;
            wait_count = 0;
            state = S_REFRESH_WAIT;
            break;

        case S_REFRESH_WAIT:
            if (wait_count >= (T_RFC - 1))
            {
                wait_count = 0;
                state = S_IDLE;
            }
            else
                wait_count = wait_count + 1;
            break;

        default:
            state = S_PWRUP;
            wait_count = 0;
            refresh_count = 0;
            initialized = false;
            ddr_cke = false;
            break;
        }

        wait();
    }
}

#include "ddr_memory_model.h"
#include <stdio.h>

ddr_memory_model::command_t ddr_memory_model::decode_command() const
{
    if (!ddr_cke.read())
        return CMD_DESELECT;

    const bool cs  = ddr_cs_n.read();
    const bool ras = ddr_ras_n.read();
    const bool cas = ddr_cas_n.read();
    const bool we  = ddr_we_n.read();

    if (cs)                    return CMD_DESELECT;
    if ( ras &&  cas &&  we)   return CMD_NOP;
    if (!ras &&  cas &&  we)   return CMD_ACTIVE;
    if ( ras && !cas &&  we)   return CMD_READ;
    if ( ras && !cas && !we)   return CMD_WRITE;
    if (!ras &&  cas && !we)   return CMD_PRECHARGE;
    if (!ras && !cas &&  we)   return CMD_REFRESH;
    if (!ras && !cas && !we)   return CMD_MRS;
    if ( ras &&  cas && !we)   return CMD_BURST_TERMINATE;

    return CMD_UNKNOWN;
}

void ddr_memory_model::report_timing_error(error_t code,
                                           const char *msg,
                                           int b)
{
    timing_error.write(true);
    timing_error_count.write(timing_error_count.read() + 1);
    last_error_code.write((unsigned int)code);

    if (b >= 0)
        printf("DDR TIMING ERROR @ cycle %lu: bank %d: %s\n",
               cycle, b, msg);
    else
        printf("DDR TIMING ERROR @ cycle %lu: %s\n",
               cycle, msg);
}

void ddr_memory_model::schedule_read_auto_precharge(int b)
{
    // READ with A10=1:
    // internal precharge cannot begin until both the minimum active time
    // and the read-to-precharge requirement have been satisfied.
    unsigned long by_tras = bank[b].active_cycle + T_RAS;
    unsigned long by_rtp  = cycle + T_RTP;

    bank[b].auto_precharge_cycle =
        (by_tras > by_rtp) ? by_tras : by_rtp;

    bank[b].auto_precharge_pending = true;
}

void ddr_memory_model::schedule_write_auto_precharge(int b)
{
    // This is scheduled when the final 32-bit controller-side write word
    // is observed. In this abstract model that marks the end of the BL4 data.
    unsigned long by_tras = bank[b].active_cycle + T_RAS;
    unsigned long by_twr  = cycle + T_WR;

    bank[b].auto_precharge_cycle =
        (by_tras > by_twr) ? by_tras : by_twr;

    bank[b].auto_precharge_pending = true;
}

void ddr_memory_model::service_auto_precharge()
{
    for (int b = 0; b < 4; ++b) {
        if (bank[b].auto_precharge_pending &&
            cycle >= bank[b].auto_precharge_cycle) {

            bank[b].open = false;
            bank[b].auto_precharge_pending = false;
            bank[b].last_precharge_cycle = cycle;

            printf("DDR MODEL: auto-precharge bank %d @ cycle %lu\n",
                   b, cycle);
        }
    }
}

void ddr_memory_model::update_debug_outputs()
{
    sc_uint<4> open_mask = 0;
    sc_uint<4> ap_mask = 0;

    for (int b = 0; b < 4; ++b) {
        open_mask[b] = bank[b].open;
        ap_mask[b] = bank[b].auto_precharge_pending;
    }

    bank_open_mask.write(open_mask);
    auto_precharge_mask.write(ap_mask);
}

unsigned long ddr_memory_model::make_key(sc_uint<2> b,
                                         sc_uint<13> r,
                                         sc_uint<9> c) const
{
    // BL4-aligned column.
    unsigned long col = ((unsigned long)c.to_uint()) & ~3UL;

    return (((unsigned long)r.to_uint()) << 11) |
           (((unsigned long)b.to_uint()) << 9) |
           col;
}

sc_uint<64> ddr_memory_model::read_mem(sc_uint<2> b,
                                       sc_uint<13> r,
                                       sc_uint<9> c)
{
    unsigned long key = make_key(b, r, c);

    std::map<unsigned long, sc_uint<64> >::iterator it =
        memory.find(key);

    if (it == memory.end())
        return 0;

    return it->second;
}

void ddr_memory_model::write_mem(sc_uint<2> b,
                                 sc_uint<13> r,
                                 sc_uint<9> c,
                                 sc_uint<64> data,
                                 sc_uint<8> mask)
{
    unsigned long key = make_key(b, r, c);

    sc_uint<64> oldv = read_mem(b, r, c);
    sc_uint<64> newv = oldv;

    // Mask convention used by the current controller:
    // mask bit 0 = write byte, mask bit 1 = preserve byte.
    for (int byte = 0; byte < 8; ++byte) {
        if (((mask >> byte) & 1) == 0) {
            newv.range(byte * 8 + 7, byte * 8) =
                data.range(byte * 8 + 7, byte * 8);
        }
    }

    memory[key] = newv;
}

void ddr_memory_model::main_thread()
{
    phy_rd_valid.write(false);
    phy_rd_data.write(0);

    timing_error.write(false);
    timing_error_count.write(0);
    last_error_code.write(ERR_NONE);

    bank_open_mask.write(0);
    auto_precharge_mask.write(0);

    cycle = 0;

    last_activate_any = 0;
    last_refresh = 0;
    have_last_activate = false;
    have_last_refresh = false;

    write_pending = false;
    write_auto_precharge = false;
    write_words_seen = 0;

    read_pending = false;
    read_return_cycle = 0;
    read_word_index = 0;

    memory.clear();

    for (int i = 0; i < 4; ++i)
        bank[i] = bank_state_t();

    wait();

    while (true) {
        ++cycle;

        timing_error.write(false);
        last_error_code.write(ERR_NONE);
        phy_rd_valid.write(false);

        // Internal DRAM events that mature on this CK.
        service_auto_precharge();

        // Simplified CL2.5 read return:
        // first controller posedge at/after the physical 2.5 CK point.
        if (read_pending && cycle >= read_return_cycle) {
            phy_rd_valid.write(true);

            if (read_word_index == 0) {
                phy_rd_data.write(read_burst.range(31, 0));
                read_word_index = 1;
            } else {
                phy_rd_data.write(read_burst.range(63, 32));
                read_word_index = 0;
                read_pending = false;
            }
        }

        // Capture BL4 x16 as two 32-bit controller-side words.
        if (write_pending && phy_wr_valid.read()) {
            if (write_words_seen == 0) {
                write_word0 = phy_wr_data.read();
                write_mask0 = phy_wr_mask.read();
                write_words_seen = 1;
            } else {
                sc_uint<64> burst;
                sc_uint<8> mask;

                burst.range(31, 0)  = write_word0;
                burst.range(63, 32) = phy_wr_data.read();

                mask.range(3, 0) = write_mask0;
                mask.range(7, 4) = phy_wr_mask.read();

                write_mem(write_bank, write_row, write_col,
                          burst, mask);

                if (write_auto_precharge)
                    schedule_write_auto_precharge(
                        (int)write_bank.to_uint());

                write_pending = false;
                write_auto_precharge = false;
                write_words_seen = 0;
            }
        }

        command_t cmd = decode_command();

        if (cmd == CMD_ACTIVE) {
            int b = (int)ddr_ba.read().to_uint();

            if (bank[b].open || bank[b].auto_precharge_pending) {
                report_timing_error(
                    ERR_BANK_NOT_CLOSED,
                    "ACTIVE issued before previous row/precharge completed",
                    b);
            }

            if (bank[b].last_precharge_cycle != 0 &&
                (cycle - bank[b].last_precharge_cycle) < T_RP) {
                report_timing_error(
                    ERR_TRP,
                    "tRP violation: ACTIVE too soon after PRECHARGE",
                    b);
            }

            if (bank[b].has_activated &&
                (cycle - bank[b].active_cycle) < T_RC) {
                report_timing_error(
                    ERR_TRC,
                    "tRC violation: repeated ACTIVE too soon",
                    b);
            }

            if (have_last_activate &&
                (cycle - last_activate_any) < T_RRD) {
                report_timing_error(
                    ERR_TRRD,
                    "tRRD violation: ACTIVE commands too close",
                    b);
            }

            bank[b].open = true;
            bank[b].has_activated = true;
            bank[b].row = ddr_addr.read().range(12, 0);
            bank[b].active_cycle = cycle;
            bank[b].auto_precharge_pending = false;

            last_activate_any = cycle;
            have_last_activate = true;
        }
        else if (cmd == CMD_READ || cmd == CMD_WRITE) {
            int b = (int)ddr_ba.read().to_uint();

            if (!bank[b].open) {
                report_timing_error(
                    ERR_BANK_CLOSED,
                    "READ/WRITE issued to closed bank",
                    b);
            }
            else if (bank[b].auto_precharge_pending) {
                report_timing_error(
                    ERR_AP_PENDING,
                    "READ/WRITE issued while auto-precharge is pending",
                    b);
            }
            else if ((cycle - bank[b].active_cycle) < T_RCD) {
                report_timing_error(
                    ERR_TRCD,
                    "tRCD violation: READ/WRITE too soon after ACTIVE",
                    b);
            }

            sc_uint<9> col = ddr_addr.read().range(8, 0);
            bool auto_precharge = ddr_addr.read()[10];

            if (cmd == CMD_WRITE) {
                write_pending = true;
                write_auto_precharge = auto_precharge;
                write_bank = b;
                write_row = bank[b].row;
                write_col = col;
                write_words_seen = 0;
                bank[b].last_write_cycle = cycle;
            }
            else {
                read_burst = read_mem(b, bank[b].row, col);
                read_pending = true;
                read_return_cycle = cycle + READ_RETURN_DELAY;
                read_word_index = 0;

                if (auto_precharge)
                    schedule_read_auto_precharge(b);
            }
        }
        else if (cmd == CMD_PRECHARGE) {
            bool all_banks = ddr_addr.read()[10];

            for (int b = 0; b < 4; ++b) {
                if (!all_banks &&
                    b != (int)ddr_ba.read().to_uint())
                    continue;

                if (bank[b].open) {
                    if ((cycle - bank[b].active_cycle) < T_RAS) {
                        report_timing_error(
                            ERR_TRAS,
                            "tRAS violation: PRECHARGE too soon after ACTIVE",
                            b);
                    }

                    if (bank[b].last_write_cycle != 0 &&
                        (cycle - bank[b].last_write_cycle) < T_WR) {
                        report_timing_error(
                            ERR_TWR,
                            "tWR violation: PRECHARGE too soon after WRITE",
                            b);
                    }
                }

                bank[b].open = false;
                bank[b].auto_precharge_pending = false;
                bank[b].last_precharge_cycle = cycle;
            }
        }
        else if (cmd == CMD_REFRESH) {
            for (int b = 0; b < 4; ++b) {
                if (bank[b].open ||
                    bank[b].auto_precharge_pending) {
                    report_timing_error(
                        ERR_REFRESH_OPEN,
                        "AUTO REFRESH issued before bank precharge completed",
                        b);
                }

                if (bank[b].last_precharge_cycle != 0 &&
                    (cycle - bank[b].last_precharge_cycle) < T_RP) {
                    report_timing_error(
                        ERR_REFRESH_TRP,
                        "tRP violation: REFRESH too soon after PRECHARGE",
                        b);
                }
            }

            if (have_last_refresh &&
                (cycle - last_refresh) < T_RFC) {
                report_timing_error(
                    ERR_TRFC,
                    "tRFC violation: REFRESH commands too close",
                    -1);
            }

            last_refresh = cycle;
            have_last_refresh = true;
        }

        update_debug_outputs();
        wait();
    }
}

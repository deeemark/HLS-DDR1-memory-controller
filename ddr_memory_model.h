#ifndef DDR_MEMORY_MODEL_H
#define DDR_MEMORY_MODEL_H

#include <systemc.h>
#include <map>

// Behavioral DDR1 x16 memory/timing model.
//
// Verification model only:
//   - decodes DDR command pins
//   - tracks bank/row state
//   - checks controller-level DDR timing
//   - models BL4 x16 backing storage
//   - models auto-precharge
//

SC_MODULE(ddr_memory_model)
{
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    sc_in<bool> ddr_cke;
    sc_in<bool> ddr_cs_n;
    sc_in<bool> ddr_ras_n;
    sc_in<bool> ddr_cas_n;
    sc_in<bool> ddr_we_n;
    sc_in< sc_uint<2> > ddr_ba;
    sc_in< sc_uint<14> > ddr_addr;

    sc_in<bool> phy_wr_valid;
    sc_in< sc_uint<32> > phy_wr_data;
    sc_in< sc_uint<4> > phy_wr_mask;

    sc_out<bool> phy_rd_valid;
    sc_out< sc_uint<32> > phy_rd_data;

    sc_out<bool> timing_error;
    sc_out< sc_uint<16> > timing_error_count;
    sc_out< sc_uint<8> > last_error_code;

    // Debug signals useful in waveforms and unit testing.
    sc_out< sc_uint<4> > bank_open_mask;
    sc_out< sc_uint<4> > auto_precharge_mask;

    // 6 ns CK controller-cycle abstraction.
    enum {
        T_RCD = 4,
        T_RP  = 4,
        T_RAS = 8,
        T_RC  = 11,
        T_RFC = 13,
        T_RRD = 3,
        T_WR  = 3,
        T_RTP = 2,
        READ_RETURN_DELAY = 3
    };

    enum command_t {
        CMD_DESELECT,
        CMD_NOP,
        CMD_ACTIVE,
        CMD_READ,
        CMD_WRITE,
        CMD_PRECHARGE,
        CMD_REFRESH,
        CMD_MRS,
        CMD_BURST_TERMINATE,
        CMD_UNKNOWN
    };

    enum error_t {
        ERR_NONE              = 0,
        ERR_BANK_NOT_CLOSED   = 1,
        ERR_TRP               = 2,
        ERR_TRC               = 3,
        ERR_TRRD              = 4,
        ERR_BANK_CLOSED       = 5,
        ERR_AP_PENDING        = 6,
        ERR_TRCD              = 7,
        ERR_TRAS              = 8,
        ERR_TWR               = 9,
        ERR_REFRESH_OPEN      = 10,
        ERR_REFRESH_TRP       = 11,
        ERR_TRFC              = 12
    };

    struct bank_state_t {
        bool open;
        bool has_activated;
        sc_uint<13> row;

        unsigned long active_cycle;
        unsigned long last_precharge_cycle;
        unsigned long last_write_cycle;

        bool auto_precharge_pending;
        unsigned long auto_precharge_cycle;

        bank_state_t()
            : open(false),
              has_activated(false),
              row(0),
              active_cycle(0),
              last_precharge_cycle(0),
              last_write_cycle(0),
              auto_precharge_pending(false),
              auto_precharge_cycle(0)
        {}
    };

    bank_state_t bank[4];

    unsigned long cycle;
    unsigned long last_activate_any;
    unsigned long last_refresh;
    bool have_last_activate;
    bool have_last_refresh;

    std::map<unsigned long, sc_uint<64> > memory;

    bool write_pending;
    bool write_auto_precharge;
    sc_uint<2> write_bank;
    sc_uint<13> write_row;
    sc_uint<9> write_col;
    sc_uint<32> write_word0;
    sc_uint<4> write_mask0;
    int write_words_seen;

    bool read_pending;
    unsigned long read_return_cycle;
    sc_uint<64> read_burst;
    int read_word_index;

    void main_thread();

    command_t decode_command() const;
    void report_timing_error(error_t code, const char *msg, int b);

    void schedule_read_auto_precharge(int b);
    void schedule_write_auto_precharge(int b);
    void service_auto_precharge();
    void update_debug_outputs();

    unsigned long make_key(sc_uint<2> b, sc_uint<13> r, sc_uint<9> c) const;
    sc_uint<64> read_mem(sc_uint<2> b, sc_uint<13> r, sc_uint<9> c);
    void write_mem(sc_uint<2> b, sc_uint<13> r, sc_uint<9> c,
                   sc_uint<64> data, sc_uint<8> mask);

    SC_CTOR(ddr_memory_model)
    {
        SC_CTHREAD(main_thread, clk.pos());
        reset_signal_is(rst_n, false);
    }
};

#endif

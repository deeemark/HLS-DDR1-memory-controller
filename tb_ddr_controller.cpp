#include "tb_ddr_controller.h"

static int is_precharge(bool cs, bool ras, bool cas, bool we)
{
    return (!cs && !ras && cas && !we);
}

static int is_refresh(bool cs, bool ras, bool cas, bool we)
{
    return (!cs && !ras && !cas && we);
}

static int is_mrs(bool cs, bool ras, bool cas, bool we)
{
    return (!cs && !ras && !cas && !we);
}

static int is_active(bool cs, bool ras, bool cas, bool we)
{
    return (!cs && !ras && cas && we);
}

static int is_read(bool cs, bool ras, bool cas, bool we)
{
    return (!cs && ras && !cas && we);
}

static int is_write(bool cs, bool ras, bool cas, bool we)
{
    return (!cs && ras && !cas && !we);
}

void test_DDR_CONTROLLER::send()
{
    unsigned int op;
    unsigned int address;
    unsigned int data_hi;
    unsigned int data_lo;
    unsigned int mask;

    sc_uint<64> data64;

    req_valid.write(false);
    req_write.write(false);
    req_addr.write(0);
    req_wdata.write(0);
    req_wmask.write(0);

    in_ddr_file = fopen(INFILENAME, "rt");

    if (!in_ddr_file)
    {
        cout << "Could not open " << INFILENAME << "\n";
        sc_stop();
        exit(-1);
    }

    wait();

    while (!init_done.read())
        wait();

    while (fscanf(in_ddr_file, "%u %u %u %u %u",
                  &op, &address, &data_hi, &data_lo, &mask) != EOF)
    {
        while (!req_ready.read())
            wait();

        data64 = 0;
        data64.range(31,0)  = data_lo;
        data64.range(63,32) = data_hi;

        req_write.write(op ? true : false);
        req_addr.write(address);
        req_wdata.write(data64);
        req_wmask.write(mask);
        req_valid.write(true);

        wait();

        req_valid.write(false);

        while (req_ready.read())
            wait();

        while (!req_ready.read())
            wait();
    }

    fclose(in_ddr_file);

    wait();
    wait();

    test_done = true;

    wait();
    wait();

    cout << endl << "Starting comparing results " << endl;

    compare_results();
    sc_stop();

    while (true)
        wait();
}

void test_DDR_CONTROLLER::recv()
{
    bool previous_init_done;

    previous_init_done = false;

    out_ddr_file = fopen(OUTFILENAME, "wt");

    if (!out_ddr_file)
    {
        cout << "Could not open " << OUTFILENAME << "\n";
        sc_stop();
        exit(-1);
    }

    wait();

    while (true)
    {
        if (!test_done)
        {
            bool cs;
            bool ras;
            bool cas;
            bool we;

            unsigned int bank;
            unsigned int address;

            cs      = ddr_cs_n.read();
            ras     = ddr_ras_n.read();
            cas     = ddr_cas_n.read();
            we      = ddr_we_n.read();
            bank    = ddr_ba.read().to_uint();
            address = ddr_addr.read().to_uint();

            if (is_precharge(cs, ras, cas, we))
            {
                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_PRECHARGE, bank, address);
            }
            else if (is_refresh(cs, ras, cas, we))
            {
                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_REFRESH, bank, address);
            }
            else if (is_mrs(cs, ras, cas, we))
            {
                if (bank == 1)
                    fprintf(out_ddr_file, "%d %u %u\n",
                            EVT_EMRS, bank, address);
                else
                    fprintf(out_ddr_file, "%d %u %u\n",
                            EVT_MRS, bank, address);
            }
            else if (is_active(cs, ras, cas, we))
            {
                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_ACTIVE, bank, address);
            }
            else if (is_write(cs, ras, cas, we))
            {
                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_WRITE, bank, address);
            }
            else if (is_read(cs, ras, cas, we))
            {
                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_READ, bank, address);
            }

            if (!previous_init_done && init_done.read())
                fprintf(out_ddr_file, "%d 0 0\n", EVT_INIT_DONE);

            if (phy_wr_valid.read())
            {
                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_PHY_WRITE,
                        phy_wr_data.read().to_uint(),
                        phy_wr_mask.read().to_uint());
            }

            if (rsp_valid.read())
            {
                sc_uint<64> value;

                value = rsp_rdata.read();

                fprintf(out_ddr_file, "%d %u %u\n",
                        EVT_RESPONSE,
                        value.range(31,0).to_uint(),
                        value.range(63,32).to_uint());
            }

            previous_init_done = init_done.read();
        }
        else
        {
            fflush(out_ddr_file);
        }

        wait();
    }
}

void test_DDR_CONTROLLER::compare_results()
{
    int event_out;
    int event_golden;

    unsigned int value0_out;
    unsigned int value1_out;
    unsigned int value0_golden;
    unsigned int value1_golden;

    int line;
    int errors;
    int output_status;

    line = 1;
    errors = 0;

    fflush(out_ddr_file);
    fclose(out_ddr_file);

    out_ddr_file_read = fopen(OUTFILENAME, "rt");

    if (!out_ddr_file_read)
    {
        cout << "Could not open " << OUTFILENAME << endl;
        sc_stop();
        exit(-1);
    }

    out_ddr_golden_file = fopen(OUTFILENAME_GOLDEN, "rt");

    if (!out_ddr_golden_file)
    {
        cout << "Could not open " << OUTFILENAME_GOLDEN << endl;
        sc_stop();
        exit(-1);
    }

    diff_file = fopen(DIFFFILENAME, "wt");

    if (!diff_file)
    {
        cout << "Could not open " << DIFFFILENAME << endl;
        sc_stop();
        exit(-1);
    }

    while (fscanf(out_ddr_golden_file, "%d %u %u",
                  &event_golden,
                  &value0_golden,
                  &value1_golden) != EOF)
    {
        output_status = fscanf(out_ddr_file_read, "%d %u %u",
                               &event_out,
                               &value0_out,
                               &value1_out);

        if (output_status == EOF)
        {
            cout << "\nOutput ended early at line " << line << endl;
            fprintf(diff_file,
                    "Output ended early at line %d\n",
                    line);
            errors++;
            break;
        }

        cout << endl
             << "Event[" << line << "]: "
             << event_golden << " "
             << value0_golden << " "
             << value1_golden
             << " -- "
             << event_out << " "
             << value0_out << " "
             << value1_out;

        if ((event_out  != event_golden) ||
            (value0_out != value0_golden) ||
            (value1_out != value1_golden))
        {
            cout << "\nOutput mismatch [line:" << line << "]";

            fprintf(diff_file,
                    "Output mismatch [line:%d] Golden: %d %u %u -- Output: %d %u %u\n",
                    line,
                    event_golden,
                    value0_golden,
                    value1_golden,
                    event_out,
                    value0_out,
                    value1_out);

            errors++;
        }

        line++;
    }

    if (fscanf(out_ddr_file_read, "%d %u %u",
               &event_out,
               &value0_out,
               &value1_out) != EOF)
    {
        cout << "\nSimulation generated additional unexpected events" << endl;
        fprintf(diff_file,
                "Simulation generated additional unexpected events beginning with: %d %u %u\n",
                event_out, value0_out, value1_out);
        errors++;
    }

    if (ddr_timing_error_count.read().to_uint() != 0)
    {
        cout << endl
             << "DDR memory model detected "
             << ddr_timing_error_count.read().to_uint()
             << " timing error(s); last error code = "
             << ddr_last_error_code.read().to_uint()
             << endl;

        fprintf(diff_file,
                "DDR memory model detected %u timing error(s); last error code = %u\n",
                ddr_timing_error_count.read().to_uint(),
                ddr_last_error_code.read().to_uint());

        errors++;
    }

    if (errors == 0)
        cout << endl << "Finished simulation SUCCESSFULLY" << endl;
    else
        cout << endl << "Finished simulation FAILED with "
             << errors
             << " output/timing error(s)"
             << endl;

    fclose(out_ddr_file_read);
    fclose(out_ddr_golden_file);
    fclose(diff_file);
}

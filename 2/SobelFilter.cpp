#include <cmath>
#include <vector>
#include "SobelFilter.h"
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace std;

SobelFilter::SobelFilter(sc_module_name n) : sc_module(n), t_skt("t_skt"), base_offset(0)
{
  SC_THREAD(do_filter);

  t_skt.register_b_transport(this, &SobelFilter::blocking_transport);
}

/*// sobel mask
const int mask[MASK_N][MASK_X][MASK_Y] = {{{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}},

                                          {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}}};
*/
void SobelFilter::do_filter()
{
  int check = 0;
  int avg_r;
  int avg_g;
  int avg_b;

  int temp_r[9] = {0};
  int temp_g[9] = {0};
  int temp_b[9] = {0};
  while (true)
  {
    int width = 256;
    int buffer_r[6] = {0};
    int buffer_g[6] = {0};
    int buffer_b[6] = {0};

    avg_r = 0;
    avg_g = 0;
    avg_b = 0;
    // buffer move

    for (unsigned int v = 0; v < MASK_Y; ++v)
    {
      for (unsigned int u = 0; u < MASK_X; ++u)
      {
        // if check==0 then we read 9pixl
        if (check == 0)
        {
          unsigned char tttr = i_r.read();
          unsigned char tttg = i_g.read();
          unsigned char tttb = i_b.read();
          int width = i_img_width.read() * 8; // 讀取

          temp_r[u + 3 * v] = tttr; // 讀取
          temp_g[u + 3 * v] = tttg;
          temp_b[u + 3 * v] = tttb;
        }
        else // check!=0 then we only read 3pixl
        {
          if (u == 2)
          {
            unsigned char tttr = i_r.read();
            unsigned char tttg = i_g.read();
            unsigned char tttb = i_b.read();
            int width = i_img_width.read() * 8; // 讀取

            temp_r[u + 3 * v] = tttr; // 讀取
            temp_g[u + 3 * v] = tttg;
            temp_b[u + 3 * v] = tttb;
          }
        }
        // if it is center we dont read it still 0
        if (u == 1 && v == 1)
        {
          avg_r = avg_r;
          avg_g = avg_g;
          avg_b = avg_b;
        }
        else // sum
        {
          avg_r = temp_r[u + 3 * v] + avg_r;
          avg_g = temp_g[u + 3 * v] + avg_g;
          avg_b = temp_b[u + 3 * v] + avg_b;
        }
      }
    }

    check = check + 1;
    if (check == width) // if already jump to next row then check==0;
    {
      check = 0;
    }
    if (check != 0)
    {
      buffer_r[0] = temp_r[1];
      buffer_g[0] = temp_g[1];
      buffer_b[0] = temp_b[1];

      buffer_r[1] = temp_r[2];
      buffer_g[1] = temp_g[2];
      buffer_b[1] = temp_b[2];

      buffer_r[2] = temp_r[4];
      buffer_g[2] = temp_g[4];
      buffer_b[2] = temp_b[4];

      buffer_r[3] = temp_r[5];
      buffer_g[3] = temp_g[5];
      buffer_b[3] = temp_b[5];

      buffer_r[4] = temp_r[7];
      buffer_g[4] = temp_g[7];
      buffer_b[4] = temp_b[7];

      buffer_r[5] = temp_r[8];
      buffer_g[5] = temp_g[8];
      buffer_b[5] = temp_b[8];
    }

    sort(begin(temp_r), begin(temp_r) + 9);
    sort(begin(temp_g), begin(temp_g) + 9);
    sort(begin(temp_b), begin(temp_b) + 9);

    avg_r = (avg_r + 2 * temp_r[4]) / 10;
    avg_g = (avg_g + 2 * temp_g[4]) / 10;
    avg_b = (avg_b + 2 * temp_b[4]) / 10;
    if (check != 0)
    {
      temp_r[0] = buffer_r[0];
      temp_g[0] = buffer_g[0];
      temp_b[0] = buffer_b[0];

      temp_r[1] = buffer_r[1];
      temp_g[1] = buffer_g[1];
      temp_b[1] = buffer_b[1];

      temp_r[3] = buffer_r[2];
      temp_g[3] = buffer_g[2];
      temp_b[3] = buffer_b[2];

      temp_r[4] = buffer_r[3];
      temp_g[4] = buffer_g[3];
      temp_b[4] = buffer_b[3];

      temp_r[6] = buffer_r[4];
      temp_g[6] = buffer_g[4];
      temp_b[6] = buffer_b[4];

      temp_r[7] = buffer_r[5];
      temp_g[7] = buffer_g[5];
      temp_b[7] = buffer_b[5];
    }

    unsigned char char_avg_r = avg_r;
    unsigned char char_avg_g = avg_g;
    unsigned char char_avg_b = avg_b;

    o_avg_r.write(char_avg_r);
    o_avg_g.write(char_avg_g); // 寫出
    o_avg_b.write(char_avg_b);
    // emulate module delay
  }
}

// 問題 下面基本看不懂
void SobelFilter::blocking_transport(tlm::tlm_generic_payload &payload,
                                     sc_core::sc_time &delay)
{
  sc_dt::uint64 addr = payload.get_address();
  addr = addr - base_offset;
  unsigned char *mask_ptr = payload.get_byte_enable_ptr();
  unsigned char *data_ptr = payload.get_data_ptr();
  word buffer;
  switch (payload.get_command())
  {
  case tlm::TLM_READ_COMMAND: // 做完filter後，Testbench要從Filter讀資料
    switch (addr)
    {
    case SOBEL_FILTER_RESULT_ADDR:
      buffer.uc[0] = o_avg_r.read();
      buffer.uc[1] = o_avg_g.read();
      buffer.uc[2] = o_avg_b.read();
      buffer.uc[3] = 0;
      break;
    case SOBEL_FILTER_CHECK_ADDR:
      buffer.uint = o_avg_r.num_available();
      break;
    default:
      /*std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;*/
      // debug
      cout << "we have trouble" << endl;
      break;
    }
    data_ptr[0] = buffer.uc[0];
    data_ptr[1] = buffer.uc[1];
    data_ptr[2] = buffer.uc[2];
    data_ptr[3] = buffer.uc[3];
    delay = sc_time(1, SC_NS);
    break;

  case tlm::TLM_WRITE_COMMAND: // 還沒做filter，Testbench要寫資料給Filter
    switch (addr)
    {
    case SOBEL_FILTER_R_ADDR:
      if (mask_ptr[0] == 0xff)
      {
        i_r.write(data_ptr[0]);
      }
      if (mask_ptr[1] == 0xff)
      {
        i_g.write(data_ptr[1]);
      }
      if (mask_ptr[2] == 0xff)
      {
        i_b.write(data_ptr[2]);
      }
      if (mask_ptr[3] == 0xff)
      {
        i_img_width.write(data_ptr[3]);
      }
      break;
    default:
      /*std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;*/
      // debug
      cout << "we have trouble" << endl;
      break;
    }
    delay = sc_time(1, SC_NS);
    break;

  case tlm::TLM_IGNORE_COMMAND:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  default:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }
  payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
}

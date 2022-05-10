/**
 * Copyright (c) 2020, Inc.
 * All rights reserved.
 * @Author:
 * @Mail:
 */

#include "rtsp_client/hevc_sps.h"

#include "hobotlog/hobotlog.hpp"

class NALStream {
 public:
  NALStream()
      : m_data(NULL),
        m_len(0),
        m_idx(0),
        m_bits(0),
        m_byte(0),
        m_zeros(0) {
        }
  NALStream(void *data, int len) { Init(data, len); }
  void Init(void *data, int len) {
    m_data = (unsigned char *)data;
    m_len = len;
    m_idx = 0;
    m_bits = 0;
    m_byte = 0;
    m_zeros = 0;
  }
  unsigned char GetBYTE() {
    if (m_idx >= m_len) return 0;
    unsigned char b = m_data[m_idx++];
    if (b == 0) {
      m_zeros++;
      if ((m_idx < m_len) && (m_zeros == 2) && (m_data[m_idx] == 0x03)) {
        m_idx++;
        m_zeros = 0;
      }
    } else {
      m_zeros = 0;
    }

    return b;
  }

  uint32 GetBit() {
    if (m_bits == 0) {
      m_byte = GetBYTE();
      m_bits = 8;
    }
    m_bits--;
    return (m_byte >> m_bits) & 0x1;
  }

  uint32 GetWord(int bits) {
    uint32 u = 0;
    while (bits > 0) {
      u <<= 1;
      u |= GetBit();
      bits--;
    }
    return u;
  }

  uint32 GetUE() {
    int zeros = 0;
    while (m_idx < m_len && GetBit() == 0) zeros++;
    return GetWord(zeros) + ((1 << zeros) - 1);
  }

  int GetSE() {
    uint32 UE = GetUE();
    bool positive = UE & 1;
    int SE = (UE + 1) >> 1;
    if (!positive) {
      SE = -SE;
    }
    return SE;
  }

 private:
  unsigned char *m_data;
  int m_len;
  int m_idx;
  int m_bits;
  unsigned char m_byte;
  int m_zeros;
};

bool parse_sps(unsigned char *data, int size, vc_params_t &params) {
  if (size < 20) {
    LOGE << "H265 parse sps info fail, size less than 20";
    return false;
  }
  NALStream bs(data, size);
  // seq_parameter_set_rbsp()
  bs.GetWord(4);  // sps_video_parameter_set_id
  int sps_max_sub_layers_minus1 = bs.GetWord(3) + 1;
  if (sps_max_sub_layers_minus1 > 6) {
    LOGE << "H265 parse sps info fail, sps sub layers less than 6";
    return false;
  }

  bs.GetWord(1);
  {
    bs.GetWord(2);
    bs.GetWord(1);
    params.profile = bs.GetWord(5);
    bs.GetWord(32);                //
    bs.GetWord(1);                 //
    bs.GetWord(1);                 //
    bs.GetWord(1);                 //
    bs.GetWord(1);                 //
    bs.GetWord(44);                //
    params.level = bs.GetWord(8);  // general_level_idc
    uint8 sub_layer_profile_present_flag[6] = {0};
    uint8 sub_layer_level_present_flag[6] = {0};
    for (int i = 0; i < sps_max_sub_layers_minus1; i++) {
      sub_layer_profile_present_flag[i] = bs.GetWord(1);
      sub_layer_level_present_flag[i] = bs.GetWord(1);
    }
    if (sps_max_sub_layers_minus1 > 0) {
      for (int i = sps_max_sub_layers_minus1; i < 8; i++) {
        uint8 reserved_zero_2bits = bs.GetWord(2);
        LOGD << "reserved_zero_2bits:" << reserved_zero_2bits;
      }
    }
    for (int i = 0; i < sps_max_sub_layers_minus1; i++) {
      if (sub_layer_profile_present_flag[i]) {
        bs.GetWord(2);
        bs.GetWord(1);
        bs.GetWord(5);
        bs.GetWord(32);
        bs.GetWord(1);
        bs.GetWord(1);
        bs.GetWord(1);
        bs.GetWord(1);
        bs.GetWord(44);
      }
      if (sub_layer_level_present_flag[i]) {
        bs.GetWord(8);  // sub_layer_level_idc[i]
      }
    }
  }
  uint32 sps_seq_parameter_set_id = bs.GetUE();
  if (sps_seq_parameter_set_id > 16) {
    LOGE << "H265 parse sps info, sps set id more than 15";
    // return false;
  }
  uint32 chroma_format_idc = bs.GetUE();
  if (sps_seq_parameter_set_id > 3U) {
    LOGE << "H265 parse sps info, sps set id more than 3U!";
    // return false;
  }
  if (chroma_format_idc == 3) {
    bs.GetWord(1);  //
  }

  params.width = bs.GetUE();   // pic_width_in_luma_samples
  params.height = bs.GetUE();  // pic_height_in_luma_samples
  if (bs.GetWord(1)) {
    bs.GetUE();
    bs.GetUE();
    bs.GetUE();
    bs.GetUE();
  }
  uint32 bit_depth_luma_minus8 = bs.GetUE();
  uint32 bit_depth_chroma_minus8 = bs.GetUE();
  if (bit_depth_luma_minus8 != bit_depth_chroma_minus8) {
    LOGE << "H265 parse sps info, bit_depth luma != chroma";
    // return false;
  }

  return true;
}

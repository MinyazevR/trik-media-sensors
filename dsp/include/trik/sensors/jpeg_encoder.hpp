#ifndef TRIK_SENSORS_JPEG_ENCODER_HPP_
#define TRIK_SENSORS_JPEG_ENCODER_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <trik/sensors/cv_algorithms.hpp>

#include <stdint.h>
#include <string.h>

#include <c6x.h>
#include <cassert>
#include <cmath>

/*
  JPEG encoder ported to C++ and optimized by Jake and Dmitry, www.trikset.com, 03/2015
  (originally derived from an Adobe Systems Incorporated reference encoder,
   see the original trik-media-sensors-dsp repository for the copyright notice).

  This is a baseline (DCT + quantization + Huffman) JPEG encoder that runs on the
  C674x DSP. It encodes a two-plane NV16 (Y + interleaved U/V) image into a
  standalone JPEG bitstream.
*/

/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace sensors /* **** **** **** **** **** */ {

struct BitString {
  int16_t len;
  uint16_t val;
};

static const int s_zigZag[64] = {
     0, 1, 5, 6,14,15,27,28,
     2, 4, 7,13,16,26,29,42,
     3, 8,12,17,25,30,41,43,
     9,11,18,24,31,40,44,53,
    10,19,23,32,39,45,52,54,
    20,22,33,38,46,51,55,60,
    21,34,37,47,50,56,59,61,
    35,36,48,49,57,58,62,63};

static const int s_yqt[64] = {
      16, 11, 10, 16, 24, 40, 51, 61,
      12, 12, 14, 19, 26, 58, 60, 55,
      14, 13, 16, 24, 40, 57, 69, 56,
      14, 17, 22, 29, 51, 87, 80, 62,
      18, 22, 37, 56, 68,109,103, 77,
      24, 35, 55, 64, 81,104,113, 92,
      49, 64, 78, 87,103,121,120,101,
      72, 92, 95, 98,112,100,103, 99};

static const int s_uvqt[64] = {
      17, 18, 24, 47, 99, 99, 99, 99,
      18, 21, 26, 66, 99, 99, 99, 99,
      24, 26, 56, 99, 99, 99, 99, 99,
      47, 66, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99};

static const int s_std_dc_luminance_nrcodes[17] = {0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const int s_std_dc_luminance_values[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const int s_std_ac_luminance_nrcodes[17] = {0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const int s_std_ac_luminance_values[162] = {
  0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,
  0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
  0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,
  0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
  0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,
  0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
  0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,
  0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
  0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,
  0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
  0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,
  0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
  0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,
  0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
  0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,
  0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
  0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,
  0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
  0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,
  0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
  0xf9,0xfa};

static const int s_std_dc_chrominance_nrcodes[17] = {0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const int s_std_dc_chrominance_values[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static const int s_std_ac_chrominance_nrcodes[17] = {0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const int s_std_ac_chrominance_values[162] = {
  0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,
  0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
  0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,
  0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
  0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,
  0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
  0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,
  0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
  0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,
  0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
  0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,
  0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
  0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,
  0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
  0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,
  0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
  0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,
  0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
  0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,
  0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
  0xf9,0xfa};

static const double s_aasf[8] = {
      1.0, 1.387039845, 1.306562965, 1.175875602,
      1.0, 0.785694958, 0.541196100, 0.275899379
    };

/* Shared, computed-once tables (do not depend on image quality). */
static bool s_jpegTablesReady = false;
static BitString s_bitcode[65535];
static int s_category[65535];
static BitString s_ydcHt[12];
static BitString s_uvdcHt[12];
static BitString s_yacHt[256];
static BitString s_uvacHt[256];

static void computeHuffmanTable(BitString* _ht, const int* _nrcodes, const int* _stdTable) {
  int codevalue = 0;
  int posInTable = 0;

  for (int k = 1; k <= 16; k++) {
    for (int j = 1; j <= _nrcodes[k]; j++) {
      _ht[_stdTable[posInTable]].val = codevalue;
      _ht[_stdTable[posInTable]].len = k;
      posInTable++;
      codevalue++;
    }
    codevalue *= 2;
  }
}

static void initJpegCategoryTable() {
  int nrlower = 1;
  int nrupper = 2;
  int nr;

  for (uint8_t cat = 1; cat <= 15; cat++) {
    // Positive numbers
    for (nr = nrlower; nr < nrupper; nr++) {
      s_category[32767 + nr] = cat;
      s_bitcode[32767 + nr].len = cat;
      s_bitcode[32767 + nr].val = nr;
    }
    // Negative numbers
    for (nr = -(nrupper - 1); nr <= -nrlower; nr++) {
      s_category[32767 + nr] = cat;
      s_bitcode[32767 + nr].len = cat;
      s_bitcode[32767 + nr].val = nrupper - 1 + nr;
    }
    nrlower <<= 1;
    nrupper <<= 1;
  }
}

static void initJpegTables() {
  if (s_jpegTablesReady)
    return;

  computeHuffmanTable(s_ydcHt, s_std_dc_luminance_nrcodes, s_std_dc_luminance_values);
  computeHuffmanTable(s_uvdcHt, s_std_dc_chrominance_nrcodes, s_std_dc_chrominance_values);
  computeHuffmanTable(s_yacHt, s_std_ac_luminance_nrcodes, s_std_ac_luminance_values);
  computeHuffmanTable(s_uvacHt, s_std_ac_chrominance_nrcodes, s_std_ac_chrominance_values);
  initJpegCategoryTable();

  s_jpegTablesReady = true;
}

/**
 * Class that converts a YUV422/NV16 image into a valid baseline JPEG.
 */
class JPGEncoder
{
  private:
  // http://stackoverflow.com/questions/17035464/a-fast-method-to-round-a-double-to-a-32-bit-int-explained
  static inline int double2int(double d)
  {
    d += 6755399441055744.0;
    return reinterpret_cast<int&>(d);
  }

  // Static table initialization
  int YTable[64];
  int UVTable[64];
  double fdtbl_Y[64];
  double fdtbl_UV[64];

  bool ifBlackAndWhite;

  // IO state
  uint8_t* byteoutPtr;
  int byteoutCnt;
  int bytenew;
  int bytepos;
  uint32_t m_outCapacity = 0;
  bool m_overrun = false;

  int DU[64];
  int YDU[64];
  int UDU[64];
  int VDU[64];

  void initQuantTables(int sf)
  {
    int i;
    int t;

    for (i = 0; i < 64; i++) {
      t = (s_yqt[i] * sf + 50) / 100;
      if (t < 1)
        t = 1;
      else if (t > 255)
        t = 255;
      YTable[s_zigZag[i]] = t;
    }

    for (i = 0; i < 64; i++) {
      t = (s_uvqt[i] * sf + 50) / 100;
      if (t < 1)
        t = 1;
      else if (t > 255)
        t = 255;
      UVTable[s_zigZag[i]] = t;
    }

    i = 0;
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        fdtbl_Y[i]  = (1.0 / (YTable [s_zigZag[i]] * s_aasf[row] * s_aasf[col] * 8.0));
        fdtbl_UV[i] = (1.0 / (UVTable[s_zigZag[i]] * s_aasf[row] * s_aasf[col] * 8.0));
        i++;
      }
    }
  }

  void writeBits(BitString const& bs)
  {
    uint32_t value = bs.val;

    bytepos -= bs.len;
    value <<= (16 + bytepos);
    bytenew |= value >> 16;

    if (bytepos <= 0) {
      writeByte(bytenew);
      if (bytenew == 0xFF) {
        writeByte(0);
      }
      bytepos = -bytepos;

      if (bytepos >= 8) {
        uint8_t tmp = (value >> 8) & 0xFF;
        writeByte(tmp);
        if (tmp == 0xFF) {
          writeByte(0);
        }
        bytepos -= 8;
      } else {
        value >>= 8;
      }

      bytepos = 8 - bytepos;
      bytenew = value & 0xFF;
    }
  }

  void writeByte(int value)
  {
    if (byteoutCnt >= static_cast<int>(m_outCapacity)) {
      m_overrun = true;
      return;
    }
    byteoutPtr[byteoutCnt++] = static_cast<uint8_t>(value);
  }

  void writeWord(uint16_t value)
  {
    writeByte((value >> 8) & 0xFF);
    writeByte((value    ) & 0xFF);
  }

  // DCT & quantization core

  void fDCTQuant(int* data, const double* fdtbl)
  {
    int tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
    int tmp10, tmp11, tmp12, tmp13;
    double z1, z2, z3, z4, z5, z11, z13;
    int i;
    /* Pass 1: process rows. */
    int dataOff = 0;
    #pragma MUST_ITERATE(8,8,8)
    for (i = 0; i < 8; i++) {
      tmp0 = data[dataOff + 0] + data[dataOff + 7];
      tmp7 = data[dataOff + 0] - data[dataOff + 7];
      tmp1 = data[dataOff + 1] + data[dataOff + 6];
      tmp6 = data[dataOff + 1] - data[dataOff + 6];
      tmp2 = data[dataOff + 2] + data[dataOff + 5];
      tmp5 = data[dataOff + 2] - data[dataOff + 5];
      tmp3 = data[dataOff + 3] + data[dataOff + 4];
      tmp4 = data[dataOff + 3] - data[dataOff + 4];

      /* Even part */
      tmp10 = tmp0 + tmp3;    /* phase 2 */
      tmp13 = tmp0 - tmp3;
      tmp11 = tmp1 + tmp2;
      tmp12 = tmp1 - tmp2;

      data[dataOff + 0] = tmp10 + tmp11; /* phase 3 */
      data[dataOff + 4] = tmp10 - tmp11;

      z1 = (tmp12 + tmp13) * 0.707106781; /* c4 */
      data[dataOff + 2] = tmp13 + z1; /* phase 5 */
      data[dataOff + 6] = tmp13 - z1;

      /* Odd part */
      tmp10 = tmp4 + tmp5; /* phase 2 */
      tmp11 = tmp5 + tmp6;
      tmp12 = tmp6 + tmp7;

      /* The rotator is modified from fig 4-8 to avoid extra negations. */
      z5 = (tmp10 - tmp12) * 0.382683433; /* c6 */
      z2 = 0.541196100 * tmp10 + z5; /* c2-c6 */
      z4 = 1.306562965 * tmp12 + z5; /* c2+c6 */
      z3 = tmp11 * 0.707106781; /* c4 */

      z11 = tmp7 + z3;  /* phase 5 */
      z13 = tmp7 - z3;

      data[dataOff + 5] = z13 + z2;     /* phase 6 */
      data[dataOff + 3] = z13 - z2;
      data[dataOff + 1] = z11 + z4;
      data[dataOff + 7] = z11 - z4;

      dataOff += 8; /* advance pointer to next row */
    }

    /* Pass 2: process columns. */
    dataOff = 0;
    #pragma MUST_ITERATE(8,8,8)
    for (i = 0; i < 8; i++) {
      tmp0 = data[dataOff +  0] + data[dataOff + 56];
      tmp7 = data[dataOff +  0] - data[dataOff + 56];
      tmp1 = data[dataOff +  8] + data[dataOff + 48];
      tmp6 = data[dataOff +  8] - data[dataOff + 48];
      tmp2 = data[dataOff + 16] + data[dataOff + 40];
      tmp5 = data[dataOff + 16] - data[dataOff + 40];
      tmp3 = data[dataOff + 24] + data[dataOff + 32];
      tmp4 = data[dataOff + 24] - data[dataOff + 32];

      /* Even part */
      tmp10 = tmp0 + tmp3;    /* phase 2 */
      tmp13 = tmp0 - tmp3;
      tmp11 = tmp1 + tmp2;
      tmp12 = tmp1 - tmp2;

      data[dataOff +  0] = tmp10 + tmp11; /* phase 3 */
      data[dataOff + 32] = tmp10 - tmp11;

      z1 = (tmp12 + tmp13) * 0.707106781; /* c4 */
      data[dataOff + 16] = tmp13 + z1; /* phase 5 */
      data[dataOff + 48] = tmp13 - z1;

      /* Odd part */
      tmp10 = tmp4 + tmp5; /* phase 2 */
      tmp11 = tmp5 + tmp6;
      tmp12 = tmp6 + tmp7;

      /* The rotator is modified from fig 4-8 to avoid extra negations. */
      z5 = (tmp10 - tmp12) * 0.382683433; /* c6 */
      z2 = 0.541196100 * tmp10 + z5; /* c2-c6 */
      z4 = 1.306562965 * tmp12 + z5; /* c2+c6 */
      z3 = tmp11 * 0.707106781; /* c4 */

      z11 = tmp7 + z3;  /* phase 5 */
      z13 = tmp7 - z3;

      data[dataOff + 40] = z13 + z2; /* phase 6 */
      data[dataOff + 24] = z13 - z2;
      data[dataOff +  8] = z11 + z4;
      data[dataOff + 56] = z11 - z4;

      dataOff++; /* advance pointer to next column */
    }

    // Quantize/descale the coefficients
    #pragma MUST_ITERATE(64,64,64)
    for (i = 0; i < 64; i++) {
      // Apply the quantization and scaling factor & round to nearest integer
      data[i] = double2int(data[i] * fdtbl[i]);
    }
  }

  // Chunk writing

  void writeAPP0()
  {
    writeWord(0xFFE0); // marker
    writeWord(16); // length
    writeByte(0x4A); // J
    writeByte(0x46); // F
    writeByte(0x49); // I
    writeByte(0x46); // F
    writeByte(0); // = "JFIF",'\0'
    writeByte(1); // versionhi
    writeByte(1); // versionlo
    writeByte(0); // xyunits
    writeWord(1); // xdensity
    writeWord(1); // ydensity
    writeByte(0); // thumbnwidth
    writeByte(0); // thumbnheight
  }

  void writeSOF0(int width, int height)
  {
    writeWord(0xFFC0); // marker
    if (ifBlackAndWhite)
      writeWord(11);
    else
      writeWord(17);   // length, truecolor YUV JPG

    writeByte(8);    // precision
    writeWord(height);
    writeWord(width);
    if (ifBlackAndWhite)
      writeByte(1);
    else
      writeByte(3);    // nrofcomponents

    writeByte(1);    // IdY
    writeByte(0x11); // HVY
    writeByte(0);    // QTY
    if (!ifBlackAndWhite) {
      writeByte(2);    // IdU
      writeByte(0x11); // HVU
      writeByte(1);    // QTU
      writeByte(3);    // IdV
      writeByte(0x11); // HVV
      writeByte(1);    // QTV
    }
  }

  void writeDQT()
  {
    writeWord(0xFFDB); // marker
    if (ifBlackAndWhite)
      writeWord(67);
    else
      writeWord(132);    // length
    writeByte(0);
    int i;
    for (i = 0; i < 64; i++) {
      writeByte(YTable[i]);
    }
    if (!ifBlackAndWhite) {
      writeByte(1);
      for (i = 0; i < 64; i++) {
        writeByte(UVTable[i]);
      }
    }
  }

  void writeDHT()
  {
    writeWord(0xFFC4); // marker
    if (ifBlackAndWhite)
      writeWord(0xD2);
    else
      writeWord(0x01A2); // length
    int i;

    writeByte(0); // HTYDCinfo
    #pragma MUST_ITERATE(32, 32, 32)
    for (i = 0; i < 16; i++) {
      writeByte(s_std_dc_luminance_nrcodes[i + 1]);
    }
    #pragma MUST_ITERATE(12, 12, 12)
    for (i = 0; i <= 11; i++) {
      writeByte(s_std_dc_luminance_values[i]);
    }

    writeByte(0x10); // HTYACinfo
    #pragma MUST_ITERATE(16, 16, 16)
    for (i = 0; i < 16; i++) {
      writeByte(s_std_ac_luminance_nrcodes[i + 1]);
    }
    for (i = 0; i <= 161; i++) {
      writeByte(s_std_ac_luminance_values[i]);
    }
    if (!ifBlackAndWhite) {
      writeByte(1); // HTUDCinfo
      #pragma MUST_ITERATE(16, 16, 16)
      for (i = 0; i < 16; i++) {
        writeByte(s_std_dc_chrominance_nrcodes[i + 1]);
      }
      #pragma MUST_ITERATE(12, 12, 12)
      for (i = 0; i <= 11; i++) {
        writeByte(s_std_dc_chrominance_values[i]);
      }

      writeByte(0x11); // HTUACinfo
      #pragma MUST_ITERATE(16, 16, 16)
      for (i = 0; i < 16; i++) {
        writeByte(s_std_ac_chrominance_nrcodes[i + 1]);
      }
      for (i = 0; i <= 161; i++) {
        writeByte(s_std_ac_chrominance_values[i]);
      }
    }
  }

  void writeSOS()
  {
    writeWord(0xFFDA); // marker
    if (ifBlackAndWhite) {
      writeWord(8); // length
      writeByte(1); // nrofcomponents
    } else {
      writeWord(12); // length
      writeByte(3); // nrofcomponents
    }
    writeByte(1); // IdY
    writeByte(0); // HTY
    if (!ifBlackAndWhite) {
      writeByte(2); // IdU
      writeByte(0x11); // HTU
      writeByte(3); // IdV
      writeByte(0x11); // HTV
    }
    writeByte(0); // Ss
    writeByte(0x3f); // Se
    writeByte(0); // Bf
  }

  int processDU(const int* CDU, const double* fdtbl, int DC,
                const BitString* HTDC, const BitString* HTAC)
  {
    BitString EOB = HTAC[0x00];
    BitString M16zeroes = HTAC[0xF0];
    int i;

    int duDct[64];
    memcpy(duDct, CDU, sizeof(duDct));
    fDCTQuant(duDct, fdtbl);

    // ZigZag reorder
    #pragma UNROLL(2)
    #pragma MUST_ITERATE(64, ,64)
    for (i = 0; i < 64; i++) {
      DU[s_zigZag[i]] = duDct[i];
    }
    int Diff = DU[0] - DC;
    DC = DU[0];

    // Encode DC
    if (Diff == 0) {
      writeBits(HTDC[0x00]);
    } else {
      writeBits(HTDC[s_category[32767 + Diff]]);
      writeBits(s_bitcode[32767 + Diff]);
    }

    // Encode ACs
    int end0pos = 63;
    for (; (end0pos > 0) && (DU[end0pos] == 0); end0pos--) {}
    if (end0pos == 0) {
      writeBits(EOB);
      return DC;
    }

    for (i = 1; i <= end0pos; i++) {
      int startpos = i;
      for (; (DU[i] == 0) && (i <= end0pos); i++) { }
      int nrzeroes = i - startpos;
      if (nrzeroes >= 16) {
        for (int nrmarker = 1; nrmarker <= nrzeroes / 16; nrmarker++) {
          writeBits(M16zeroes);
        }
        nrzeroes = static_cast<int>(nrzeroes & 0xF);
      }

      writeBits(HTAC[nrzeroes * 16 + s_category[32767 + DU[i]]]);
      writeBits(s_bitcode[32767 + DU[i]]);
    }
    if (end0pos != 63) {
      writeBits(EOB);
    }

    return DC;
  }

  // YCbCr NV16 - img format
  // Two planes: Y plane and CbCr combined plane
  // Decodes one 8x8 macroblock from an NV16 image. The Y and the interleaved
  // U/V planes are addressed with the same @p lineLength stride (the V4L2
  // bytesperline), exactly like CvAlgorithm::convertImageNV16ToHsv does, so a
  // padded frame never drifts out of sync. Chroma is 4:2:2 (U = high byte,
  // V = low byte of each interleaved 16-bit word).
  void getBlock(const uint8_t* img, int xpos, int ypos, int width, int height, int lineLength)
  {
    const uint8_t* restrict yPlane = img;
    const uint8_t* restrict uvPlane = img + lineLength * height;

    int pos = 0;
    #pragma MUST_ITERATE(8, ,8)
    for (int y = 0; y < 8; y++) {
      const int rowOffset = (ypos + y) * lineLength;
      #pragma MUST_ITERATE(8, ,8)
      for (int x = 0; x < 8; x++) {
        const int px = xpos + x;
        const uint16_t uv = *reinterpret_cast<const uint16_t*>(uvPlane + rowOffset + (px / 2) * 2);
        YDU[pos] = yPlane[rowOffset + px] - 128;
        UDU[pos] = static_cast<uint8_t>(uv >> 8) - 128;
        VDU[pos] = static_cast<uint8_t>(uv) - 128;
        pos++;
      }
    }
  }

  public:
  void init(uint8_t quality, bool ifBnW)
  {
    ifBlackAndWhite = ifBnW;

    if (quality <= 0)
      quality = 1;
    if (quality > 100)
      quality = 100;
    int sf;
    if (quality < 50)
      sf = static_cast<int>(5000 / quality);
    else
      sf = static_cast<int>(200 - quality * 2);

    initJpegTables();
    initQuantTables(sf);
  }

  /**
   * Created a JPEG image from the specified NV16 image.
   *
   * @param image NV16 image: Y plane (width*height) followed by interleaved U/V plane.
   * @return number of bytes written into imageout.
   */
  uint32_t encode(const uint8_t* image, int width, int height, int lineLength,
                  uint8_t* imageout, uint32_t maxOut)
  {
    // Initialize bit writer
    byteoutPtr = imageout;
    byteoutCnt = 0;
    m_outCapacity = maxOut;
    m_overrun = false;

    bytenew = 0;
    bytepos = 8;

    // Add JPEG headers
    writeWord(0xFFD8); // SOI
    writeAPP0();
    writeDQT();
    writeSOF0(width, height);
    writeDHT();
    writeSOS();

    // Encode 8x8 macroblocks
    int DCY = 0;
    int DCU = 0;
    int DCV = 0;
    bytenew = 0;
    bytepos = 8;

    for (int ypos = 0; ypos < height; ypos += 8) {
      for (int xpos = 0; xpos < width; xpos += 8) {
        getBlock(image, xpos, ypos, width, height, lineLength);
        DCY = processDU(YDU, fdtbl_Y, DCY, s_ydcHt, s_yacHt);
        if (!ifBlackAndWhite) {
          DCU = processDU(UDU, fdtbl_UV, DCU, s_uvdcHt, s_uvacHt);
          DCV = processDU(VDU, fdtbl_UV, DCV, s_uvdcHt, s_uvacHt);
        }
      }
    }

    // Do the bit alignment of the EOI marker
    if (bytepos >= 0) {
      BitString fillbits;

      fillbits.len = bytepos;
      fillbits.val = (1 << (bytepos)) - 1;

      writeBits(fillbits);
    }

    writeWord(0xFFD9); // EOI

    return m_overrun ? 0 : static_cast<uint32_t>(byteoutCnt);
  }
};

class JpegEncoderCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  JPGEncoder jpgEncoder;
  bool m_isYuyv;

  static int8_t s_nv16Buffer[IMG_WIDTH * IMG_HEIGHT * 2];

  void convertYuyvToNv16(const ImageBuffer& _inImage, int8_t* _dst) {
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t lineLength = m_inImageDesc.m_lineLength;
    uint8_t* dstY = reinterpret_cast<uint8_t*>(_dst);
    uint16_t* dstUV = reinterpret_cast<uint16_t*>(_dst + width * height);

    for (uint32_t row = 0; row < height; row++) {
      const uint8_t* srcRow = reinterpret_cast<const uint8_t*>(_inImage.m_ptr) + row * lineLength;
      for (uint32_t col = 0; col < width; col += 2) {
        const uint8_t y0 = srcRow[col * 2 + 0];
        const uint8_t u  = srcRow[col * 2 + 1];
        const uint8_t y1 = srcRow[col * 2 + 2];
        const uint8_t v  = srcRow[col * 2 + 3];
        dstY[row * width + col + 0] = y0;
        dstY[row * width + col + 1] = y1;
        dstUV[(row * width + col) / 2] = (static_cast<uint16_t>(u) << 8) | v;
      }
    }
  }

public:
  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    (void)_fastRam;
    (void)_fastRamSize;
    m_inImageDesc = _inImageDesc;
    m_outImageDesc = _outImageDesc;

    if (m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height % 4 != 0)
      return false;

    if (_inImageDesc.m_format == VideoFormat::YUV422)
      m_isYuyv = true;
    else if (_inImageDesc.m_format == VideoFormat::NV16)
      m_isYuyv = false;
    else
      return false;

    return true;
  }

  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& _inArgs, trik_cv_algorithm_out_args& _outArgs) {
    // The input frame is written to DDR by the host (uncached, via /dev/mem
    // O_SYNC) and is not covered by the rpmsg MessageQ cache handling. The
    // sensor algorithms re-read through their big scratch buffers, so the stale
    // cache lines get thrashed out naturally; the JPEG encoder reads the frame
    // directly, so it must explicitly invalidate the data caches first. Use
    // Cache_Type_ALLD (L1D+L2D, no program cache) to keep the cost low.
    Cache_inv(_inImage.m_ptr, _inImage.m_size, Cache_Type_ALLD, TRUE);

    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t lineLength = m_inImageDesc.m_lineLength;

    // NV16 has two planes (Y + interleaved U/V) of `lineLength` bytes per row;
    // YUYV is a single plane. Validate that the whole frame is available.
    const uint64_t requiredInput = static_cast<uint64_t>(height) * lineLength * (m_isYuyv ? 1u : 2u);
    if (requiredInput > _inImage.m_size)
      return false;

    const uint8_t* src = reinterpret_cast<const uint8_t*>(_inImage.m_ptr);
    uint32_t srcStride = lineLength;
    if (m_isYuyv) {
      convertYuyvToNv16(_inImage, s_nv16Buffer);
      src = reinterpret_cast<const uint8_t*>(s_nv16Buffer);
      srcStride = width; // the internal NV16 buffer is tightly packed
    }

    jpgEncoder.init(_inArgs.jpeg_image_quality, _inArgs.if_black_and_white);
    const uint32_t size = jpgEncoder.encode(src, static_cast<int>(width), static_cast<int>(height),
                                            static_cast<int>(srcStride),
                                            reinterpret_cast<uint8_t*>(_outImage.m_ptr),
                                            static_cast<uint32_t>(_outImage.m_size));
    if (size == 0 || size > _outImage.m_size)
      return false;

    _outImage.m_size = size;
    _outArgs.jpeg_size = size;

    Cache_wbInv(_outImage.m_ptr, size, Cache_Type_ALL, TRUE);

    return true;
  }
};

int8_t JpegEncoderCvAlgorithm::s_nv16Buffer[IMG_WIDTH * IMG_HEIGHT * 2];

} /* **** **** **** **** **** * namespace sensors * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */

#endif // !TRIK_SENSORS_JPEG_ENCODER_HPP_

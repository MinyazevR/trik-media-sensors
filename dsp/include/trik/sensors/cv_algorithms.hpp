#ifndef TRIK_SENSORS_CV_ALGORITHMS_HPP_
#define TRIK_SENSORS_CV_ALGORITHMS_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <c6x.h>
#include <cassert>
#include <cmath>
#include <stdint.h>

#include "image.hpp"
#include <trik/sensors/video_format.h>
#include <trik/sensors/cv_algorithm_args.h>
#include <ti/sysbios/family/c64p/Cache.h>

namespace trik {
namespace sensors {

template <typename _T>
inline _T range(_T _min, _T _val, _T _max) {
  if (_val < _min)
    return _min;
  else if (_val > _max)
    return _max;
  else
    return _val;
}

inline uint16_t pop(uint16_t x) {
  x = x - ((x >> 1) & 0x5555);
  x = (x & 0x3333) + ((x >> 2) & 0x3333);
  x = (x + (x >> 4)) & 0x0f0f;
  x = x + (x >> 8);
  x = x + (x >> 16);

  return x & 0x003f;
}

inline int makeValueRange(int _val, int _adj, int _min, int _max) {
  _val += _adj;
  if (_val > _max)
    return _max;
  else if (_val < _min)
    return _min;
  else
    return _val;
}

inline int makeValueWrap(int _val, int _adj, int _min, int _max) {
  _val += _adj;
  while (_val > _max)
    _val -= (_max - _min + 1);
  while (_val < _min)
    _val += (_max - _min + 1);

  return _val;
}

template <VideoFormat _inFormat, VideoFormat _outFormat>
class CvAlgorithm {
public:
  // CvAlgorithm(const CvAlgorithm& another) = delete;
  // CvAlgorithm& operator=(const CvAlgorithm& another) = delete;

  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) = 0;
  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& in_args, trik_cv_algorithm_out_args& out_args) = 0;

  virtual ~CvAlgorithm() {}

protected:
  typedef void (CvAlgorithm::*ConvertFuncPtr)(const ImageBuffer&);
  ConvertFuncPtr convertImageFormatToHSV = nullptr;

  ImageDesc m_inImageDesc;
  ImageDesc m_outImageDesc;

  static uint64_t s_rgb888hsv[IMG_WIDTH * IMG_HEIGHT];
  static uint32_t s_wi2wo[IMG_WIDTH];
  static uint32_t s_hi2ho[IMG_HEIGHT];

  static uint16_t* restrict s_mult43_div;
  static uint16_t* restrict s_mult255_div;

  static void __attribute__((always_inline)) writeOutputPixel(uint16_t* restrict _rgb565ptr, const uint32_t _rgb888) {
    *_rgb565ptr = ((_rgb888 >> 3) & 0x001f) | ((_rgb888 >> 5) & 0x07e0) | ((_rgb888 >> 8) & 0xf800);
  }

  void __attribute__((always_inline)) drawOutputPixelBound(const int32_t _srcCol, const int32_t _srcRow, const int32_t _srcColBot, const int32_t _srcColTop,
    const int32_t _srcRowBot, const int32_t _srcRowTop, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t srcCol = range<int32_t>(_srcColBot, _srcCol, _srcColTop);
    const int32_t srcRow = range<int32_t>(_srcRowBot, _srcRow, _srcRowTop);

    const int32_t dstRow = s_hi2ho[srcRow];
    const int32_t dstCol = s_wi2wo[srcCol];

    const uint32_t dstOfs = dstRow * m_outImageDesc.m_lineLength + dstCol * sizeof(uint16_t);

    writeOutputPixel(reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstOfs), _rgb888);
  }

  void __attribute__((always_inline)) drawFatPixel(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    drawOutputPixelBound(_srcCol - 1, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - 1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - 1, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  void __attribute__((always_inline))
  drawRgbTargetCenterLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < 100; ++adj) {
      drawOutputPixelBound(_srcCol, _srcRow - adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline))
  drawRgbTargetHorizontalCenterLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < 100; ++adj) {
      drawOutputPixelBound(_srcCol - adj, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + adj, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline))
  drawOutputCircle(const int32_t _srcCol, const int32_t _srcRow, const int32_t _srcRadius, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    int32_t circleError = 1 - _srcRadius;
    int32_t circleErrorY = 1;
    int32_t circleErrorX = -2 * _srcRadius;
    int32_t circleX = _srcRadius;
    int32_t circleY = 0;

    drawOutputPixelBound(_srcCol, _srcRow + _srcRadius, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow - _srcRadius, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + _srcRadius, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - _srcRadius, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

    while (circleY < circleX) {
      if (circleError >= 0) {
        circleX -= 1;
        circleErrorX += 2;
        circleError += circleErrorX;
      }
      circleY += 1;
      circleErrorY += 2;
      circleError += circleErrorY;

      drawOutputPixelBound(_srcCol + circleX, _srcRow + circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + circleX, _srcRow - circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleX, _srcRow + circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleX, _srcRow - circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + circleY, _srcRow + circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + circleY, _srcRow - circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleY, _srcRow + circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleY, _srcRow - circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline)) drawRgbThinLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

#pragma MUST_ITERATE(4, , 4)
    for (int adj = 0; adj < m_inImageDesc.m_height; ++adj)
      drawOutputPixelBound(_srcCol, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  void __attribute__((always_inline))
  drawRgbHorizontalLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

#pragma MUST_ITERATE(32, , 32)
    for (int adj = 0; adj < m_inImageDesc.m_width; ++adj)
      drawOutputPixelBound(_srcCol + adj, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  void __attribute__((always_inline)) drawCornerHighlight(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    drawOutputPixelBound(_srcCol - 1, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - 1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - 1, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  void __attribute__((always_inline)) drawOutputFatRectangle(const int32_t _x1, const int32_t _x2, const int32_t _y1, const int32_t _y2,
    const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int32_t c = _x1; c < _x2; c++) {
      drawOutputPixelBound(c, _y1 - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y1 + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

      drawOutputPixelBound(c, _y2 - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y2, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y2 + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }

    for (int32_t r = _y1; r < _y2; r++) {
      drawOutputPixelBound(_x1 - 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x1 + 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

      drawOutputPixelBound(_x2 - 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x2, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x2 + 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline))
  drawOutputRectangle(const int32_t _x1, const int32_t _x2, const int32_t _y1, const int32_t _y2, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int32_t c = _x1; c < _x2; c++) {
      drawOutputPixelBound(c, _y1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y2, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }

    for (int32_t r = _y1; r < _y2; r++) {
      drawOutputPixelBound(_x1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x2, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  static bool __attribute__((always_inline)) detectHsvPixel(const uint32_t _hsv, const uint64_t _hsv_range, const uint32_t _hsv_expect) {
    const uint32_t u32_hsv_det = _cmpltu4(_hsv, _hill(_hsv_range)) | _cmpgtu4(_hsv, _loll(_hsv_range));

    return (u32_hsv_det == _hsv_expect);
  }

  static uint64_t __attribute__((always_inline)) convert2xYuyvToRgb888(const uint32_t _yuyv) {
    const int64_t s64_yuyv1 = _mpyu4ll(_yuyv,
      (static_cast<uint32_t>(static_cast<uint8_t>(409 / 4)) << 24) | (static_cast<uint32_t>(static_cast<uint8_t>(298 / 4)) << 16) |
        (static_cast<uint32_t>(static_cast<uint8_t>(516 / 4)) << 8) | (static_cast<uint32_t>(static_cast<uint8_t>(298 / 4))));
    const uint32_t u32_yuyv2 =
      _dotpus4(_yuyv, (static_cast<uint32_t>(static_cast<uint8_t>(-208 / 4)) << 24) | (static_cast<uint32_t>(static_cast<uint8_t>(-100 / 4)) << 8));
    const uint32_t u32_rgb_h = _add2(_packh2(0, _hill(s64_yuyv1)), (static_cast<uint32_t>(static_cast<uint16_t>(128 / 4 + (-128 * 409 - 16 * 298) / 4))));
    const uint32_t u32_rgb_l = _add2(_packlh2(u32_yuyv2, _loll(s64_yuyv1)),
      (static_cast<uint32_t>(static_cast<uint16_t>(128 / 4 + (+128 * 100 + 128 * 208 - 16 * 298) / 4) << 16)) |
        (static_cast<uint32_t>(static_cast<uint16_t>(128 / 4 + (-128 * 516 - 16 * 298) / 4))));
    const uint32_t u32_y1y1 = _pack2(_loll(s64_yuyv1), _loll(s64_yuyv1));
    const uint32_t u32_y2y2 = _pack2(_hill(s64_yuyv1), _hill(s64_yuyv1));
    const uint32_t u32_rgb_p1h = _clr(_shr2(_add2(u32_rgb_h, u32_y1y1), 6), 16, 31);
    const uint32_t u32_rgb_p1l = _shr2(_add2(u32_rgb_l, u32_y1y1), 6);
    const uint32_t u32_rgb_p2h = _clr(_shr2(_add2(u32_rgb_h, u32_y2y2), 6), 16, 31);
    const uint32_t u32_rgb_p2l = _shr2(_add2(u32_rgb_l, u32_y2y2), 6);
    const uint32_t u32_rgb_p1 = _spacku4(u32_rgb_p1h, u32_rgb_p1l);
    const uint32_t u32_rgb_p2 = _spacku4(u32_rgb_p2h, u32_rgb_p2l);
    return _itoll(u32_rgb_p2, u32_rgb_p1);
  }

  static uint32_t __attribute__((always_inline)) convertRgb888ToHsv(const uint32_t _rgb888) {
    const uint32_t u32_rgb_or16 = _unpkhu4(_rgb888);
    const uint32_t u32_rgb_gb16 = _unpklu4(_rgb888);

    const uint32_t u32_rgb_max2 = _maxu4(_rgb888, _rgb888 >> 8);
    const uint32_t u32_rgb_max = _clr(_maxu4(u32_rgb_max2, u32_rgb_max2 >> 8), 8, 31); // top 3 bytes were non-zeroes!
    const uint32_t u32_rgb_max_max = _pack2(u32_rgb_max, u32_rgb_max);

    const uint32_t u32_hsv_ooo_val_x256 = u32_rgb_max << 8; // get max in 8..15 bits
    const uint32_t u32_rgb_min2 = _minu4(_rgb888, _rgb888 >> 8);
    const uint32_t u32_rgb_min = _minu4(u32_rgb_min2, u32_rgb_min2 >> 8); // top 3 bytes are zeroes
    const uint32_t u32_rgb_delta = u32_rgb_max - u32_rgb_min;

    /* optimized by table based multiplication with power-2 divisor, simulate 255*(max-min)/max */
    const uint32_t u32_hsv_sat_x256 = s_mult255_div[u32_rgb_max] * u32_rgb_delta;

    /* optimized by table based multiplication with power-2 divisor, simulate 43*(med-min)/(max-min) */
    const uint32_t u32_hsv_hue_mult43_div = _pack2(s_mult43_div[u32_rgb_delta], s_mult43_div[u32_rgb_delta]);
    int32_t s32_hsv_hue_x256;
    const uint32_t u32_rgb_cmp = _cmpeq2(u32_rgb_max_max, u32_rgb_gb16);
    if (u32_rgb_cmp == 0)
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 0) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _packhl2(u32_rgb_gb16, u32_rgb_gb16)));
    else if (u32_rgb_cmp == 1)
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 2) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _packlh2(u32_rgb_or16, u32_rgb_gb16)));
    else // 2, 3
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 1) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _pack2(u32_rgb_gb16, u32_rgb_or16)));

    const uint32_t u32_hsv_hue_x256 = static_cast<uint32_t>(s32_hsv_hue_x256);
    const uint32_t u32_hsv_sat_hue_x256 = _pack2(u32_hsv_sat_x256, u32_hsv_hue_x256);
    const uint32_t u32_hsv = _packh4(u32_hsv_ooo_val_x256, u32_hsv_sat_hue_x256);
    return u32_hsv;
  }

    void convertImageNV16ToHsv(const ImageBuffer& _inImage)
    {
      const uint32_t srcImageRowEffectiveSize       = m_inImageDesc.m_width;
      const uint32_t srcImageRowEffectiveToFullSize = m_inImageDesc.m_lineLength - srcImageRowEffectiveSize;
      const int8_t* restrict srcImageRowY     = _inImage.m_ptr;
      const int8_t* restrict srcImageRowC     = _inImage.m_ptr + m_inImageDesc.m_lineLength*m_inImageDesc.m_height;
      const int8_t* restrict srcImageToY      = srcImageRowY + m_inImageDesc.m_lineLength*m_inImageDesc.m_height;
      uint64_t* restrict rgb888hsvptr         = s_rgb888hsv;

      assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, ,4)
      while (srcImageRowY != srcImageToY)
      {
        assert(reinterpret_cast<intptr_t>(srcImageRowY) % 8 == 0); // let's pray...
        assert(reinterpret_cast<intptr_t>(srcImageRowC) % 8 == 0); // let's pray...
        const uint32_t* restrict srcImageColY4 = reinterpret_cast<const uint32_t*>(srcImageRowY);
        const uint32_t* restrict srcImageColC4 = reinterpret_cast<const uint32_t*>(srcImageRowC);
        srcImageRowY += srcImageRowEffectiveSize;
        srcImageRowC += srcImageRowEffectiveSize;

        assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(32/4, ,32/4)
        while (reinterpret_cast<const int8_t*>(srcImageColY4) != srcImageRowY)
        {
          assert(reinterpret_cast<const int8_t*>(srcImageColC4) != srcImageRowC);

          const uint32_t yy4x = *srcImageColY4++;
          const uint32_t uv4x = _swap4(*srcImageColC4++);

          const uint32_t yuyv12 = (_unpklu4(yy4x)) | (_unpklu4(uv4x) << 8);
          const uint32_t yuyv34 = (_unpkhu4(yy4x)) | (_unpkhu4(uv4x) << 8);

          const uint64_t rgb12 = convert2xYuyvToRgb888(yuyv12);
          *rgb888hsvptr++ = _itoll(_loll(rgb12), convertRgb888ToHsv(_loll(rgb12)));
          *rgb888hsvptr++ = _itoll(_hill(rgb12), convertRgb888ToHsv(_hill(rgb12)));

          const uint64_t rgb34 = convert2xYuyvToRgb888(yuyv34);
          *rgb888hsvptr++ = _itoll(_loll(rgb34), convertRgb888ToHsv(_loll(rgb34)));
          *rgb888hsvptr++ = _itoll(_hill(rgb34), convertRgb888ToHsv(_hill(rgb34)));
        }

        srcImageRowY += srcImageRowEffectiveToFullSize;
        srcImageRowC += srcImageRowEffectiveToFullSize;
      }
    }

  // Code from old media sensors. 
  // It seems (not exactly) that it is faster by 0.003 seconds, than code below (0.015 vs 0.018), 
  // check it later in assembler.
  void convertImageYuyvToHsv(const ImageBuffer& _inImage)
    {
      const uint32_t srcImageRowEffectiveSize       = m_inImageDesc.m_width*sizeof(uint16_t);
      const uint32_t srcImageRowEffectiveToFullSize = m_inImageDesc.m_lineLength - srcImageRowEffectiveSize;
      const int8_t* restrict srcImageRowY     = _inImage.m_ptr;
      const int8_t* restrict srcImageToY      = srcImageRowY + m_inImageDesc.m_lineLength*m_inImageDesc.m_height;
      uint64_t* restrict rgb888hsvptr         = s_rgb888hsv;

      assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, ,4)
      while (srcImageRowY != srcImageToY)
      {
        assert(reinterpret_cast<intptr_t>(srcImageRowY) % 8 == 0); // let's pray...
        const uint64_t* restrict srcImageCol4 = reinterpret_cast<const uint64_t*>(srcImageRowY);
        srcImageRowY += srcImageRowEffectiveSize;

        assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(32/4, ,32/4)
        while (reinterpret_cast<const int8_t*>(srcImageCol4) != srcImageRowY)
        {
          const uint64_t yuyv2x = *srcImageCol4++;
          const uint64_t rgb12 = convert2xYuyvToRgb888(_loll(yuyv2x));
          *rgb888hsvptr++ = _itoll(_loll(rgb12), convertRgb888ToHsv(_loll(rgb12)));
          *rgb888hsvptr++ = _itoll(_hill(rgb12), convertRgb888ToHsv(_hill(rgb12)));

          const uint64_t rgb34 = convert2xYuyvToRgb888(_hill(yuyv2x));
          *rgb888hsvptr++ = _itoll(_loll(rgb34), convertRgb888ToHsv(_loll(rgb34)));
          *rgb888hsvptr++ = _itoll(_hill(rgb34), convertRgb888ToHsv(_hill(rgb34)));
        }

        srcImageRowY += srcImageRowEffectiveToFullSize;
      }
    }

//   void convertImageYuyvToHsv(const ImageBuffer& _inImage) {
//     const uint64_t* restrict src = reinterpret_cast<const uint64_t*>(_inImage.m_ptr);
//     uint64_t* restrict dst = s_rgb888hsv;
// #pragma MUST_ITERATE(32, , 32)
//     while ((reinterpret_cast<const uint8_t*> (src)) < (reinterpret_cast<const uint8_t*> (src)) + m_inImageDesc.m_width * m_inImageDesc.m_height * 2) {
//       const uint64_t yuyv2x = *src++;
//       const uint64_t rgb12 = convert2xYuyvToRgb888(_loll(yuyv2x));
//       *dst++ = _itoll(_loll(rgb12), convertRgb888ToHsv(_loll(rgb12)));
//       *dst++ = _itoll(_hill(rgb12), convertRgb888ToHsv(_hill(rgb12)));
      
//       const uint64_t rgb34 = convert2xYuyvToRgb888(_hill(yuyv2x));
//       *dst++ = _itoll(_loll(rgb34), convertRgb888ToHsv(_loll(rgb34)));
//       *dst++ = _itoll(_hill(rgb34), convertRgb888ToHsv(_hill(rgb34)));
//     }
//   }

  bool commonSetup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    m_inImageDesc = _inImageDesc;
    m_outImageDesc = _outImageDesc;

    if (m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height % 4 != 0)
      return false;
    
    if (_inImageDesc.m_format == VideoFormat::YUV422) {
      convertImageFormatToHSV = &CvAlgorithm::convertImageYuyvToHsv;
    } else if (_inImageDesc.m_format == VideoFormat::NV16) {
      convertImageFormatToHSV = &CvAlgorithm::convertImageNV16ToHsv;
    } else { 
      return false;
    }

#define min(x, y) x < y ? x : y;
    const double srcToDstShift =
      min(static_cast<double>(m_outImageDesc.m_width) / m_inImageDesc.m_width, static_cast<double>(m_outImageDesc.m_height) / m_inImageDesc.m_height);

    const uint32_t widthIn = _inImageDesc.m_width;
    uint32_t* restrict p_wi2wo = s_wi2wo;
#pragma MUST_ITERATE(4, , 4)
    for (int i = 0; i < widthIn; i++)
      *(p_wi2wo++) = i * srcToDstShift;

    const uint32_t heightIn = _inImageDesc.m_height;
    uint32_t* restrict p_hi2ho = s_hi2ho;
#pragma MUST_ITERATE(32, , 32)
    for (uint32_t i = 0; i < heightIn; i++)
      *(p_hi2ho++) = i * srcToDstShift;

    /* Static member initialization on first instance creation */
    if (s_mult43_div == NULL || s_mult255_div == NULL) {
      if (_fastRamSize < (1u << 8) * sizeof(*s_mult43_div) + (1u << 8) * sizeof(*s_mult255_div))
        return false;

      s_mult43_div = reinterpret_cast<typeof(s_mult43_div)>(_fastRam);
      _fastRam += (1u << 8) * sizeof(*s_mult43_div);
      s_mult255_div = reinterpret_cast<typeof(s_mult255_div)>(_fastRam);
      _fastRam += (1u << 8) * sizeof(*s_mult255_div);

      s_mult43_div[0] = 0;
      s_mult255_div[0] = 0;
      for (uint32_t idx = 1; idx < (1u << 8); ++idx) {
        s_mult43_div[idx] = (43u * (1u << 8)) / idx;
        s_mult255_div[idx] = (255u * (1u << 8)) / idx;
      }
    }
    return true;
  }

  CvAlgorithm() {}
};

uint64_t restrict CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X>::s_rgb888hsv[IMG_WIDTH * IMG_HEIGHT];
uint32_t restrict CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X>::s_wi2wo[IMG_WIDTH];
uint32_t restrict CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X>::s_hi2ho[IMG_HEIGHT];
uint16_t* restrict CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X>::s_mult43_div = NULL;
uint16_t* restrict CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X>::s_mult255_div = NULL;

}
}

#include "edge_line_sensor.hpp"
#include "line_sensor.hpp"
#include "motion_sensor.hpp"
#include "mxn_sensor.hpp"
#include "object_sensor.hpp"
#include "jpeg_encoder.hpp"

#endif

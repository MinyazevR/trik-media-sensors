#ifndef TRIK_SENSORS_CV_ALGORITHM_ARGS_H_
#define TRIK_SENSORS_CV_ALGORITHM_ARGS_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define TRIK_MAX_TARGET_COUNT 8
#define COLORS_NUM   9

#define COLORS_WIDTHM_MAX   3
#define COLORS_HEIGHTN_MAX  3

typedef struct MxnParams
{
  size_t m_m;
  size_t m_n;
} MxnParams;

typedef struct trik_cv_algorithm_in_args {
  uint16_t detect_hue_from; // [0..359]
  uint16_t detect_hue_to;   // [0..359]
  uint8_t detect_sat_from;  // [0..100]
  uint8_t detect_sat_to;    // [0..100]
  uint8_t detect_val_from;  // [0..100]
  uint8_t detect_val_to;    // [0..100]
  bool auto_detect_hsv;     // [true|false]
  uint8_t jpeg_image_quality; // [1..100], only for TRIK_CV_ALGORITHM_JPEG_ENCODER
  bool if_black_and_white;    // [true|false], only for TRIK_CV_ALGORITHM_JPEG_ENCODER

  union {
    MxnParams mxnParams;
  } extra_inArgs;
} trik_cv_algorithm_in_args;

typedef struct TargetColors
{
  uint32_t m_colors[COLORS_NUM]; //treeColor
} TargetColors;

typedef struct TargetLocation
{
  int16_t x;
  int16_t y;
  uint16_t size;
} TargetLocation;

typedef struct trik_cv_algorithm_out_target {
  union {
    TargetLocation targetLocation;
    TargetColors targetColors;
  } out_target;
} trik_cv_algorithm_out_target;

typedef struct trik_cv_algorithm_out_args {
  struct trik_cv_algorithm_out_target targets[TRIK_MAX_TARGET_COUNT];
  uint16_t detect_hue_from; // [0..359]
  uint16_t detect_hue_to;   // [0..359]
  uint8_t detect_sat_from;  // [0..100]
  uint8_t detect_sat_to;    // [0..100]
  uint8_t detect_val_from;  // [0..100]
  uint8_t detect_val_to;    // [0..100]
  uint32_t jpeg_size;       // [bytes], only for TRIK_CV_ALGORITHM_JPEG_ENCODER
} trik_cv_algorithm_out_args;

#if defined(__cplusplus)
}
#endif

#endif

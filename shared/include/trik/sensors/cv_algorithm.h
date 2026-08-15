#ifndef TRIK_SENSORS_CV_ALGORITHM_H_
#define TRIK_SENSORS_CV_ALGORITHM_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "cmd.h"
#include <string.h>

enum trik_cv_algorithm {
  TRIK_CV_ALGORITHM_NONE = -1,
  TRIK_CV_ALGORITHM_MOTION_SENSOR,
  TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR,
  TRIK_CV_ALGORITHM_LINE_SENSOR,
  TRIK_CV_ALGORITHM_OBJECT_SENSOR,
  TRIK_CV_ALGORITHM_MXN_SENSOR,
  TRIK_CV_ALGORITHM_JPEG_ENCODER

};

#if defined(__cplusplus)
}
#endif

#endif

#include <trik/sensors/cv_algorithms.h>

#include <trik/sensors/cv_algorithms.hpp>
#include <trik/sensors/video_format.h>

namespace trik {
namespace sensors {

int8_t fastRam[4096];

MotionSensorCvAlgorithm motionSensorCvAlgorithm;
EdgeLineSensorCvAlgorithm edgeLineSensorCvAlgorithm;
ObjectSensorCvAlgorithm objectSensorCvAlgorithm;
LineSensorCvAlgorithm lineSensorCvAlgorithm;
MxnSensorCvAlgorithm mxnSensorCvAlgorithm;
JpegEncoderCvAlgorithm jpegEncoderCvAlgorithm;

extern "C" int trik_init_cv_algorithm(enum trik_cv_algorithm algorithm, enum VideoFormat video_format, uint32_t line_length) {
  ImageDesc inDesc = {
    .m_width = IMG_WIDTH,
    .m_height = IMG_HEIGHT,
    .m_lineLength = line_length,
    .m_format = video_format,
  };
  ImageDesc outDesc = {
    .m_width = IMG_HEIGHT,
    .m_height = IMG_WIDTH,
    .m_lineLength = IMG_HEIGHT * 2,
    .m_format = VideoFormat::RGB565X,
  };
  if (algorithm == TRIK_CV_ALGORITHM_MOTION_SENSOR)
    return motionSensorCvAlgorithm.setup(inDesc, outDesc, fastRam, sizeof(fastRam) / sizeof(fastRam[0]));
  else if (algorithm == TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR)
    return edgeLineSensorCvAlgorithm.setup(inDesc, outDesc, fastRam, sizeof(fastRam) / sizeof(fastRam[0]));
  else if (algorithm == TRIK_CV_ALGORITHM_OBJECT_SENSOR)
    return objectSensorCvAlgorithm.setup(inDesc, outDesc, fastRam, sizeof(fastRam) / sizeof(fastRam[0]));
  else if (algorithm == TRIK_CV_ALGORITHM_LINE_SENSOR)
    return lineSensorCvAlgorithm.setup(inDesc, outDesc, fastRam, sizeof(fastRam) / sizeof(fastRam[0]));
  else if (algorithm == TRIK_CV_ALGORITHM_MXN_SENSOR)
    return mxnSensorCvAlgorithm.setup(inDesc, outDesc, fastRam, sizeof(fastRam) / sizeof(fastRam[0]));
  else if (algorithm == TRIK_CV_ALGORITHM_JPEG_ENCODER)
    return jpegEncoderCvAlgorithm.setup(inDesc, outDesc, fastRam, sizeof(fastRam) / sizeof(fastRam[0]));
  else
    return 0;
}

extern "C" int trik_run_cv_algorithm(enum trik_cv_algorithm algorithm, struct buffer in_buffer, struct buffer out_buffer,
  struct trik_cv_algorithm_in_args in_args, struct trik_cv_algorithm_out_args* out_args) {
  ImageBuffer inBuffer = { .m_ptr = (int8_t*) in_buffer.start, .m_size = in_buffer.length };
  ImageBuffer outBuffer = { .m_ptr = (int8_t*) out_buffer.start, .m_size = out_buffer.length };
  if (algorithm == TRIK_CV_ALGORITHM_MOTION_SENSOR)
    return motionSensorCvAlgorithm.run(inBuffer, outBuffer, in_args, *out_args);
  else if (algorithm == TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR)
    return edgeLineSensorCvAlgorithm.run(inBuffer, outBuffer, in_args, *out_args);
  else if (algorithm == TRIK_CV_ALGORITHM_OBJECT_SENSOR)
    return objectSensorCvAlgorithm.run(inBuffer, outBuffer, in_args, *out_args);
  else if (algorithm == TRIK_CV_ALGORITHM_LINE_SENSOR)
    return lineSensorCvAlgorithm.run(inBuffer, outBuffer, in_args, *out_args);
  else if (algorithm == TRIK_CV_ALGORITHM_MXN_SENSOR)
    return mxnSensorCvAlgorithm.run(inBuffer, outBuffer, in_args, *out_args);
  else if (algorithm == TRIK_CV_ALGORITHM_JPEG_ENCODER)
    return jpegEncoderCvAlgorithm.run(inBuffer, outBuffer, in_args, *out_args);
  else
    return 0;
}

}
}

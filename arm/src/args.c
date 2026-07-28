#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/videodev2.h>

#include "trik/sensors/args.h"
#include "trik/sensors/cv_algorithm.h"
#include "trik/sensors/log.h"

static enum trik_cv_algorithm trik_cv_algorithm_from_string(const char* string) {
  if (strcmp(string, "motion_sensor") == 0)
    return TRIK_CV_ALGORITHM_MOTION_SENSOR;
  if (strcmp(string, "edge_line_sensor") == 0)
    return TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR;
  if (strcmp(string, "object_sensor") == 0)
    return TRIK_CV_ALGORITHM_OBJECT_SENSOR;
  if (strcmp(string, "line_sensor") == 0)
    return TRIK_CV_ALGORITHM_LINE_SENSOR;
  if (strcmp(string, "mxn_sensor") == 0)
    return TRIK_CV_ALGORITHM_MXN_SENSOR;
  return TRIK_CV_ALGORITHM_NONE;
}

static uint32_t parse_v4l2_format(const char* str) {
  static const struct { const char* name; uint32_t fmt; } formats[] = {
    {"rgb888", V4L2_PIX_FMT_RGB24},   {"rgb565",  V4L2_PIX_FMT_RGB565},
    {"rgb565x", V4L2_PIX_FMT_RGB565X}, {"yuv444", V4L2_PIX_FMT_YUV32},
    {"yuv422", V4L2_PIX_FMT_YUYV},    {"yuv422p", V4L2_PIX_FMT_YUV422P},
    {"nv16",   V4L2_PIX_FMT_NV16},
  };
  for (size_t i = 0; i < sizeof(formats) / sizeof(formats[0]); i++)
    if (!strcasecmp(str, formats[i].name))
      return formats[i].fmt;
  return 0;
}

enum { OPT_V4L2_PATH = 256, OPT_V4L2_WIDTH, OPT_V4L2_HEIGHT, OPT_V4L2_FORMAT,
       OPT_FB_PATH, OPT_RC_FIFO_IN, OPT_RC_FIFO_OUT, OPT_VIDEO_OUT,
       OPT_SENSOR_TYPE, OPT_CONFIG_FILE, OPT_MXN_M, OPT_MXN_N, OPT_LOG_LEVEL };

static const struct option s_longopts[] = {
  {"v4l2-path",   1, NULL, OPT_V4L2_PATH},
  {"v4l2-width",  1, NULL, OPT_V4L2_WIDTH},
  {"v4l2-height", 1, NULL, OPT_V4L2_HEIGHT},
  {"v4l2-format", 1, NULL, OPT_V4L2_FORMAT},
  {"fb-path",     1, NULL, OPT_FB_PATH},
  {"rc-fifo-in",  1, NULL, OPT_RC_FIFO_IN},
  {"rc-fifo-out", 1, NULL, OPT_RC_FIFO_OUT},
  {"video-out",   1, NULL, OPT_VIDEO_OUT},
  {"sensor-type", 1, NULL, OPT_SENSOR_TYPE},
  {"config-file", 1, NULL, OPT_CONFIG_FILE},
  {"mxn-width-m", 1, NULL, OPT_MXN_M},
  {"mxn-height-n",1, NULL, OPT_MXN_N},
  {"log-level",   1, NULL, OPT_LOG_LEVEL},
  {"help",        0, NULL, 'h'},
  {NULL, 0, NULL, 0}
};

bool argsParse(Runtime* _runtime, int _argc, char* const _argv[]) {
  int opt;
  RuntimeConfig* cfg;

  if (_runtime == NULL)
    return false;

  cfg = &_runtime->m_config;

  while ((opt = getopt_long(_argc, _argv, "h", s_longopts, NULL)) != -1) {
    switch (opt) {
    case OPT_V4L2_PATH:  cfg->m_v4l2Config.m_path = optarg;                break;
    case OPT_V4L2_WIDTH: cfg->m_v4l2Config.m_width = atoi(optarg);         break;
    case OPT_V4L2_HEIGHT: cfg->m_v4l2Config.m_height = atoi(optarg);       break;
    case OPT_V4L2_FORMAT:
      cfg->m_v4l2Config.m_format = parse_v4l2_format(optarg);
      if (!cfg->m_v4l2Config.m_format) {
        LOG(LOG_ERROR, "Unknown v4l2 format '%s'", optarg);
        return false;
      }
      break;
    case OPT_FB_PATH:     cfg->m_fbConfig.m_path = optarg;                 break;
    case OPT_RC_FIFO_IN:  cfg->m_rcConfig.m_fifoInput = optarg;            break;
    case OPT_RC_FIFO_OUT: cfg->m_rcConfig.m_fifoOutput = optarg;           break;
    case OPT_VIDEO_OUT:   cfg->m_rcConfig.m_videoOutEnable = atoi(optarg); break;
    case OPT_SENSOR_TYPE: cfg->m_rcConfig.m_sensorType = trik_cv_algorithm_from_string(optarg); break;
    case OPT_CONFIG_FILE: cfg->m_configFile = optarg;                      break;
    case OPT_MXN_M:       cfg->m_rcConfig.m_extraParams.m_mxnParams.m_m = atoi(optarg); break;
    case OPT_MXN_N:       cfg->m_rcConfig.m_extraParams.m_mxnParams.m_n = atoi(optarg); break;
    case OPT_LOG_LEVEL:
      if (strcmp(optarg, "debug") == 0)      g_log_level = LOG_DEBUG;
      else if (strcmp(optarg, "info") == 0)  g_log_level = LOG_INFO;
      else if (strcmp(optarg, "warn") == 0)  g_log_level = LOG_WARN;
      else if (strcmp(optarg, "error") == 0) g_log_level = LOG_ERROR;
      else {
        LOG(LOG_ERROR, "Unknown log-level '%s'", optarg);
        return false;
      }
      break;
    case 'h':
    default:
      return false;
    }
  }

  if (cfg->m_v4l2Config.m_path == NULL) {
    LOG(LOG_ERROR, "Missing required argument: --v4l2-path");
    return false;
  }
  if (cfg->m_rcConfig.m_fifoInput == NULL) {
    LOG(LOG_ERROR, "Missing required argument: --rc-fifo-in");
    return false;
  }
  if (cfg->m_rcConfig.m_fifoOutput == NULL) {
    LOG(LOG_ERROR, "Missing required argument: --rc-fifo-out");
    return false;
  }
  if (cfg->m_rcConfig.m_sensorType < 0) {
    LOG(LOG_ERROR, "Missing required argument: --sensor-type");
    return false;
  }

  if (cfg->m_rcConfig.m_sensorType == TRIK_CV_ALGORITHM_MXN_SENSOR) {
    if (cfg->m_rcConfig.m_extraParams.m_mxnParams.m_m <= 0
        && cfg->m_rcConfig.m_extraParams.m_mxnParams.m_n <= 0) {
      LOG(LOG_ERROR, "Missing or invalid required argument: mxn-width-m or mxn-height-n");
      return false;
    }
  }

  return true;
}

void argsHelp(Runtime* _runtime, const char* _arg0) {
  if (_runtime == NULL)
    return;

  LOG(LOG_ERROR,
    "Usage:\n"
    "    %s <opts>\n"
    " where opts are:\n"
    "   --v4l2-path    <input-device-path>\n"
    "   --v4l2-width   <input-width>\n"
    "   --v4l2-height  <input-height>\n"
    "   --v4l2-format  <input-pixel-format>\n"
    "   --fb-path      <output-device-path>\n"
    "   --rc-fifo-in            <remote-control-fifo-input>\n"
    "   --rc-fifo-out           <remote-control-fifo-output>\n"
    "   --video-out             <enable-video-output>\n"
    "   --sensor-type             <type-of-sensor-algo>\n"
    "   --log-level              <error|warn|info|debug>\n"
    "   --help",
    _arg0);
}

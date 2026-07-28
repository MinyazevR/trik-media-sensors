#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "trik/sensors/arm_server.h"
#include "trik/sensors/cv_algorithm_args.h"
#include "trik/sensors/log.h"
#include "trik/sensors/runtime.h"
#include "trik/sensors/thread_input.h"
#include "trik/sensors/thread_video.h"

static const RuntimeConfig s_runtimeConfig = {
  .m_configFile = NULL,
  .m_v4l2Config = { NULL, 320, 240, V4L2_PIX_FMT_NV16 },
  .m_fbConfig = { "/dev/fb0" },
  .m_rcConfig = { NULL, NULL, TRIK_CV_ALGORITHM_NONE, true } };

void runtimeReset(Runtime* _runtime) {
  memset(_runtime, 0, sizeof(*_runtime));
  _runtime->m_config = s_runtimeConfig;
  _runtime->m_modules.m_v4l2Input.m_fd = -1;
  _runtime->m_modules.m_fbOutput.m_fd = -1;
  _runtime->m_modules.m_rcInput.m_fifoInputFd = -1;
  _runtime->m_modules.m_rcInput.m_fifoOutputFd = -1;
  _runtime->m_threads.m_terminate = true;
  pthread_mutex_init(&_runtime->m_state.m_mutex, NULL);
}

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

bool runtimeParseArgs(Runtime* _runtime, int _argc, char* const _argv[]) {
  int opt;
  int longopt;
  RuntimeConfig* cfg;

  static const char* s_optstring = "h";
  static const struct option s_longopts[] = { 
    { "v4l2-path", 1, NULL, 0 },  //0                                               
    { "v4l2-width", 1, NULL, 0 }, 
    { "v4l2-height", 1, NULL, 0 },   //2                                                        
    { "v4l2-format", 1, NULL, 0 }, 
    { "fb-path", 1, NULL, 0 }, //4
    { "rc-fifo-in", 1, NULL, 0 }, 
    { "rc-fifo-out", 1, NULL, 0 }, //6
    { "video-out", 1, NULL, 0 }, 
    { "sensor-type", 1, NULL, 0 },     //8                                                       
    { "config-file", 1, NULL, 0 }, 
    { "mxn-width-m", 1, NULL, 0 }, //10
    { "mxn-height-n", 1, NULL, 0 },             
    { "log-level", 1, NULL, 0 }, //12
    { "help", 0, NULL, 'h' }, 
       { NULL, 0, NULL, 0 } };

  if (_runtime == NULL)
    return false;

  cfg = &_runtime->m_config;

  while ((opt = getopt_long(_argc, _argv, s_optstring, s_longopts, &longopt)) != -1) {
    switch (opt) {
    case 0:
      switch (longopt) {
      case 0:
        cfg->m_v4l2Config.m_path = optarg;
        break;
      case 1:
        cfg->m_v4l2Config.m_width = atoi(optarg);
        break;
      case 2:
        cfg->m_v4l2Config.m_height = atoi(optarg);
        break;
      case 3:
        if (!strcasecmp(optarg, "rgb888"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_RGB24;
        else if (!strcasecmp(optarg, "rgb565"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_RGB565;
        else if (!strcasecmp(optarg, "rgb565x"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_RGB565X;
        else if (!strcasecmp(optarg, "yuv444"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_YUV32;
        else if (!strcasecmp(optarg, "yuv422"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_YUYV;
        else if (!strcasecmp(optarg, "yuv422p"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_YUV422P;
        else if (!strcasecmp(optarg, "nv16"))
          cfg->m_v4l2Config.m_format = V4L2_PIX_FMT_NV16;
        else {
          LOG(LOG_ERROR,
            "Unknown v4l2 format '%s'\n"
            "Known formats: rgb888, rgb565, rgb565x, yuv444, yuv422, yuv422p, nv16",
            optarg);
          return false;
        }
        break;

      case 4:
        cfg->m_fbConfig.m_path = optarg;
        break;

      case 5:
        cfg->m_rcConfig.m_fifoInput = optarg;
        break;
      case 6:
        cfg->m_rcConfig.m_fifoOutput = optarg;
        break;
      case 7:
        cfg->m_rcConfig.m_videoOutEnable = atoi(optarg);
        break;
      case 8:
        cfg->m_rcConfig.m_sensorType = trik_cv_algorithm_from_string(optarg);
        break;
      case 9:
        cfg->m_configFile = optarg;
        break;
      case 10:
        cfg->m_rcConfig.m_extraParams.m_mxnParams.m_m = atoi(optarg);
        break;
      case 11:
        cfg->m_rcConfig.m_extraParams.m_mxnParams.m_n = atoi(optarg);
        break;
      case 12:
        if (strcmp(optarg, "debug") == 0)
          g_log_level = LOG_DEBUG;
        else if (strcmp(optarg, "info") == 0)
          g_log_level = LOG_INFO;
        else if (strcmp(optarg, "warn") == 0)
          g_log_level = LOG_WARN;
        else if (strcmp(optarg, "error") == 0)
          g_log_level = LOG_ERROR;
        else {
          LOG(LOG_ERROR, "Unknown log-level '%s', expected error|warn|info|debug", optarg);
          return false;
        }
        break;
      default:
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

void runtimeArgsHelpMessage(Runtime* _runtime, const char* _arg0) {
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

int runtimeInit(Runtime* _runtime) {
  int res = 0;

  if (_runtime == NULL)
    return EINVAL;

  if ((res = v4l2InputInit()) != 0) {
    LOG(LOG_ERROR, "v4l2InputInit() failed: %d", res);
    return res;
  }

  if ((res = fbOutputInit()) != 0) {
    LOG(LOG_ERROR, "fbOutputInit() failed: %d", res);
    return res;
  }

  if ((res = rcInputInit()) != 0) {
    LOG(LOG_ERROR, "rcInputInit() failed: %d", res);
    return res;
  }

  return 0;
}

int runtimeFini(Runtime* _runtime) {
  int res;

  if (_runtime == NULL)
    return EINVAL;

  if ((res = rcInputFini()) != 0)
    LOG(LOG_ERROR, "rcInputFini() failed: %d", res);

  if ((res = fbOutputFini()) != 0)
    LOG(LOG_ERROR, "fbOutputFini() failed: %d", res);

  if ((res = v4l2InputFini()) != 0)
    LOG(LOG_ERROR, "v4l2InputFini() failed: %d", res);

  return 0;
}

int runtimeStart(Runtime* _runtime) {
  int res;
  int exit_code = 0;
  RuntimeThreads* rt;

  if (_runtime == NULL)
    return EINVAL;

  rt = &_runtime->m_threads;
  rt->m_terminate = false;

  if ((res = pthread_create(&rt->m_inputThread, NULL, &threadInput, _runtime)) != 0) {
    LOG(LOG_ERROR, "pthread_create(input) failed: %d", res);
    exit_code = res;
    goto exit;
  }

  if ((res = pthread_create(&rt->m_videoThread, NULL, &trik_start_arm_server, _runtime)) != 0) {
    LOG(LOG_ERROR, "pthread_create(arm  server) failed: %d", res);
    exit_code = res;
    goto exit_join_input_thread;
  }

  return 0;

exit_join_input_thread:
  pthread_cancel(rt->m_inputThread);
  pthread_join(rt->m_inputThread, NULL);

exit:
  runtimeSetTerminate(_runtime);
  return exit_code;
}

int runtimeStop(Runtime* _runtime) {
  RuntimeThreads* rt;

  if (_runtime == NULL)
    return EINVAL;

  rt = &_runtime->m_threads;

  runtimeSetTerminate(_runtime);
  pthread_join(rt->m_videoThread, NULL);
  pthread_join(rt->m_inputThread, NULL);

  return 0;
}

const V4L2Config* runtimeCfgV4L2Input(const Runtime* _runtime) {
  if (_runtime == NULL)
    return NULL;

  return &_runtime->m_config.m_v4l2Config;
}

const FBConfig* runtimeCfgFBOutput(const Runtime* _runtime) {
  if (_runtime == NULL)
    return NULL;

  return &_runtime->m_config.m_fbConfig;
}

const RCConfig* runtimeCfgRCInput(const Runtime* _runtime) {
  if (_runtime == NULL)
    return NULL;

  return &_runtime->m_config.m_rcConfig;
}

V4L2Input* runtimeModV4L2Input(Runtime* _runtime) {
  if (_runtime == NULL)
    return NULL;

  return &_runtime->m_modules.m_v4l2Input;
}

FBOutput* runtimeModFBOutput(Runtime* _runtime) {
  if (_runtime == NULL)
    return NULL;

  return &_runtime->m_modules.m_fbOutput;
}

RCInput* runtimeModRCInput(Runtime* _runtime) {
  if (_runtime == NULL)
    return NULL;

  return &_runtime->m_modules.m_rcInput;
}

bool runtimeGetTerminate(Runtime* _runtime) {
  if (_runtime == NULL)
    return true;
  return _runtime->m_threads.m_terminate;
}

void runtimeSetTerminate(Runtime* _runtime) {
  if (_runtime == NULL)
    return;

  _runtime->m_threads.m_terminate = true;
}

int runtimeGetTargetDetectParams(Runtime* _runtime, trik_cv_algorithm_in_args* _targetDetectParams) {
  if (_runtime == NULL || _targetDetectParams == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  *_targetDetectParams = _runtime->m_state.m_targetDetectParams;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeSetTargetDetectParams(Runtime* _runtime, const trik_cv_algorithm_in_args* _targetDetectParams) {
  if (_runtime == NULL || _targetDetectParams == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  _runtime->m_state.m_targetDetectParams = *_targetDetectParams;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeGetVideoOutParams(Runtime* _runtime, bool* _videoOutEnable) {
  if (_runtime == NULL || _videoOutEnable == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  *_videoOutEnable = _runtime->m_state.m_videoOutEnable;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeGetMxnParams(Runtime* _runtime, MxnParams* _mxnParams) {
  if (_runtime == NULL || _mxnParams == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  *_mxnParams = _runtime->m_state.extra_runtimeState.m_mxnParams;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeSetVideoOutParams(Runtime* _runtime, const bool* _videoOutEnable) {
  if (_runtime == NULL || _videoOutEnable == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  _runtime->m_state.m_videoOutEnable = *_videoOutEnable;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeFetchTargetDetectCommand(Runtime* _runtime, TargetDetectCommand* _targetDetectCommand) {
  if (_runtime == NULL || _targetDetectCommand == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  *_targetDetectCommand = _runtime->m_state.m_targetDetectCommand;
  _runtime->m_state.m_targetDetectCommand.m_cmd = 0;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeSetTargetDetectCommand(Runtime* _runtime, const TargetDetectCommand* _targetDetectCommand) {
  if (_runtime == NULL || _targetDetectCommand == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  _runtime->m_state.m_targetDetectCommand = *_targetDetectCommand;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeSetMxNParams(Runtime* _runtime, MxnParams* mxnParams) {
  if (_runtime == NULL || mxnParams == NULL)
    return EINVAL;

  pthread_mutex_lock(&_runtime->m_state.m_mutex);
  _runtime->m_state.extra_runtimeState.m_mxnParams = *mxnParams;
  pthread_mutex_unlock(&_runtime->m_state.m_mutex);
  return 0;
}

int runtimeReportTargetLocation(Runtime* _runtime, const TargetLocation* _targetLocation) {
  if (_runtime == NULL || _targetLocation == NULL)
    return EINVAL;

#warning Unsafe
  rcInputUnsafeReportTargetLocation(&_runtime->m_modules.m_rcInput, _targetLocation);

  return 0;
}

int runtimeReportTargetColors(Runtime* _runtime, const TargetColors* _targetColors) {
  if (_runtime == NULL || _targetColors == NULL)
    return EINVAL;

#warning Unsafe
  rcInputUnsafeReportTargetColors(&_runtime->m_modules.m_rcInput, _targetColors);

  return 0;
}

int runtimeReportTargetDetectParams(Runtime* _runtime, const trik_cv_algorithm_out_args* _targetDetectParams) {
  if (_runtime == NULL || _targetDetectParams == NULL)
    return EINVAL;

#warning Unsafe
  rcInputUnsafeReportTargetDetectParams(&_runtime->m_modules.m_rcInput, _targetDetectParams);

  return 0;
}

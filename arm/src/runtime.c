#include <assert.h>
#include <errno.h>
#include <stdio.h>
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

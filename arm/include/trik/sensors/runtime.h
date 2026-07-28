#ifndef TRIK_V4L2_DSP_FB_INTERNAL_RUNTIME_H_
#define TRIK_V4L2_DSP_FB_INTERNAL_RUNTIME_H_

#include <pthread.h>

#include "trik/sensors/common.h"
#include "trik/sensors/module_fb.h"
#include "trik/sensors/module_rc.h"
#include "trik/sensors/module_v4l2.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct RuntimeConfig {
  const char* m_configFile;

  V4L2Config m_v4l2Config;
  FBConfig m_fbConfig;
  RCConfig m_rcConfig;
} RuntimeConfig;

typedef struct DSP {
  struct buffer* dsp_in_buf;
  struct buffer* dsp_out_buf;
} DSP;

typedef struct RuntimeModules {
  V4L2Input m_v4l2Input;
  FBOutput m_fbOutput;
  RCInput m_rcInput;
  DSP m_dsp;
} RuntimeModules;

typedef struct RuntimeThreads {
  volatile bool m_terminate;

  pthread_t m_inputThread;
  pthread_t m_videoThread;
} RuntimeThreads;

typedef struct RuntimeState {
  pthread_mutex_t m_mutex;
  trik_cv_algorithm_in_args m_targetDetectParams;
  TargetDetectCommand m_targetDetectCommand;
  bool m_videoOutEnable;

  union {
    MxnParams   m_mxnParams;
  } extra_runtimeState;
} RuntimeState;

typedef struct Runtime {
  RuntimeConfig m_config;
  RuntimeModules m_modules;
  RuntimeThreads m_threads;
  RuntimeState m_state;

} Runtime;

void runtimeReset(Runtime* _runtime);

int runtimeInit(Runtime* _runtime);
int runtimeFini(Runtime* _runtime);
int runtimeStart(Runtime* _runtime);
int runtimeStop(Runtime* _runtime);

const V4L2Config* runtimeCfgV4L2Input(const Runtime* _runtime);
const FBConfig* runtimeCfgFBOutput(const Runtime* _runtime);
const RCConfig* runtimeCfgRCInput(const Runtime* _runtime);

V4L2Input* runtimeModV4L2Input(Runtime* _runtime);
FBOutput* runtimeModFBOutput(Runtime* _runtime);
RCInput* runtimeModRCInput(Runtime* _runtime);

bool runtimeGetTerminate(Runtime* _runtime);
void runtimeSetTerminate(Runtime* _runtime);
int runtimeGetTargetDetectParams(Runtime* _runtime, trik_cv_algorithm_in_args* _targetDetectParams);
int runtimeSetTargetDetectParams(Runtime* _runtime, const trik_cv_algorithm_in_args* _targetDetectParams);
int runtimeFetchTargetDetectCommand(Runtime* _runtime, TargetDetectCommand* _targetDetectCommand);
int runtimeSetTargetDetectCommand(Runtime* _runtime, const TargetDetectCommand* _targetDetectCommand);
int runtimeSetMxNParams(Runtime* _runtime, MxnParams* mxnParams);

int runtimeGetVideoOutParams(Runtime* _runtime, bool* _videoOutEnable);
int runtimeSetVideoOutParams(Runtime* _runtime, const bool* _videoOutEnable);

int runtimeReportTargetLocation(Runtime* _runtime, const TargetLocation* _targetLocation);
int runtimeReportTargetColors(Runtime* _runtime, const TargetColors* _targetColors);
int runtimeGetMxnParams(Runtime* _runtime, MxnParams* _mxnParams);
int runtimeReportTargetDetectParams(Runtime* _runtime, const trik_cv_algorithm_out_args* _targetDetectParams);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // !TRIK_V4L2_DSP_FB_INTERNAL_RUNTIME_H_

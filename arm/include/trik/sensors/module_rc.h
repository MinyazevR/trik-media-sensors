#ifndef TRIK_V4L2_DSP_FB_INTERNAL_MODULE_RC_H_
#define TRIK_V4L2_DSP_FB_INTERNAL_MODULE_RC_H_

#include <stdbool.h>

#include "trik/sensors/common.h"
#include "trik/sensors/cv_algorithm.h"
#include "trik/sensors/cv_algorithm_args.h"
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct RCConfig // what user wants to set
{
  const char* m_fifoInput;
  const char* m_fifoOutput;
  enum trik_cv_algorithm m_sensorType;
  bool m_videoOutEnable;
  union {
    MxnParams   m_mxnParams;
  } m_extraParams;
} RCConfig;

typedef struct MxnParamsInput
{
  MxnParams   m_mxnParams;
  bool m_mxnParamsUpdated;
} MxnParamsInput;

typedef struct RCInput {
  int m_fifoInputFd;
  char* m_fifoInputName;
  char* m_fifoInputReadBuffer;
  size_t m_fifoInputReadBufferSize;
  size_t m_fifoInputReadBufferUsed;

  int m_fifoOutputFd;
  char* m_fifoOutputName;

  bool m_targetDetectParamsUpdated;
  int m_targetDetectHue;
  int m_targetDetectHueTolerance;
  int m_targetDetectSat;
  int m_targetDetectSatTolerance;
  int m_targetDetectVal;
  int m_targetDetectValTolerance;

  bool m_targetDetectCommandUpdated;
  int m_targetDetectCommand;

  bool m_videoOutParamsUpdated;
  bool m_videoOutEnable;
  union {
    MxnParamsInput m_mxnParamsInput;
  } m_extraRCInput;
} RCInput;

int rcInputInit(void);
int rcInputFini();

int rcInputOpen(RCInput* _rc, const RCConfig* _config);
int rcInputClose(RCInput* _rc);
int rcInputStart(RCInput* _rc);
int rcInputStop(RCInput* _rc);

int rcInputReadFifoInput(RCInput* _rc);

int rcInputGetTargetDetectParams(RCInput* _rc, trik_cv_algorithm_in_args* _targetDetectParams);
int rcInputGetTargetDetectCommand(RCInput* _rc, TargetDetectCommand* _targetDetectCommand);
int rcInputGetMxNParams(RCInput* _rc, MxnParams* mxnParams);
int rcInputGetVideoOutParams(RCInput* _rc, bool* _videoOutEnable);

int rcInputUnsafeReportTargetLocation(RCInput* _rc, const TargetLocation* _targetLocation);
int rcInputUnsafeReportTargetColors(RCInput* _rc, const TargetColors* _targetColors);
int rcInputUnsafeReportTargetDetectParams(RCInput* _rc, const trik_cv_algorithm_out_args* _targetDetectParams);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // !TRIK_V4L2_DSP_FB_INTERNAL_MODULE_RC_H_

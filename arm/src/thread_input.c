#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>

#include "trik/sensors/cv_algorithm_args.h"
#include "trik/sensors/log.h"
#include "trik/sensors/module_rc.h"
#include "trik/sensors/runtime.h"
#include "trik/sensors/thread_input.h"

static int threadInputSelectLoop(Runtime* _runtime, RCInput* _rc) {
  int res;
  int maxFd = 0;
  fd_set fdsIn;
  static const struct timespec s_selectTimeout = { .tv_sec = 1, .tv_nsec = 0 };

  if (_runtime == NULL || _rc == NULL)
    return EINVAL;

  FD_ZERO(&fdsIn);

  if (_rc->m_fifoInputFd != -1) {
    FD_SET(_rc->m_fifoInputFd, &fdsIn);
    if (maxFd < _rc->m_fifoInputFd)
      maxFd = _rc->m_fifoInputFd;
  }

  if ((res = pselect(maxFd + 1, &fdsIn, NULL, NULL, &s_selectTimeout, NULL)) < 0) {
    res = errno;
    LOG(LOG_ERROR, "pselect() failed: %d", res);
    return res;
  }

  if (_rc->m_fifoInputFd != -1 && FD_ISSET(_rc->m_fifoInputFd, &fdsIn)) {
    if ((res = rcInputReadFifoInput(_rc)) != 0) {
      LOG(LOG_ERROR, "rcInputReadFifoInput() failed: %d", res);
      return res;
    }
  }

  trik_cv_algorithm_in_args targetDetectParams;
  if ((res = rcInputGetTargetDetectParams(_rc, &targetDetectParams)) != 0) {
    if (res != ENODATA) {
      LOG(LOG_ERROR, "rcInputGetTargetDetectParams() failed: %d", res);
      return res;
    }
  } else if ((res = runtimeSetTargetDetectParams(_runtime, &targetDetectParams)) != 0) {
    LOG(LOG_ERROR, "runtimeSetTargetDetectParams() failed: %d", res);
    return res;
  }

  bool videoOutEnable;
  if ((res = rcInputGetVideoOutParams(_rc, &videoOutEnable)) != 0) {
    if (res != ENODATA) {
      LOG(LOG_ERROR, "rcInputGetVideoOutParams() failed: %d", res);
      return res;
    }
  } else if ((res = runtimeSetVideoOutParams(_runtime, &videoOutEnable)) != 0) {
    LOG(LOG_ERROR, "runtimeSetVideoOutParams() failed: %d", res);
    return res;
  }

  TargetDetectCommand targetDetectCommand;
  if ((res = rcInputGetTargetDetectCommand(_rc, &targetDetectCommand)) != 0) {
    if (res != ENODATA) {
      LOG(LOG_ERROR, "rcInputGetTargetDetectCommand() failed: %d", res);
      return res;
    }
  } else if ((res = runtimeSetTargetDetectCommand(_runtime, &targetDetectCommand)) != 0) {
    LOG(LOG_ERROR, "runtimeSetTargetDetectCommand() failed: %d", res);
    return res;
  }

  MxnParams mxnParams;
  if ((res = rcInputGetMxNParams(_rc, &mxnParams)) != 0) {
    if (res != ENODATA) {
      LOG(LOG_ERROR, "rcInputGetMxNParams() failed: %d", res);
      return res;
    }
  } else if ((res = runtimeSetMxNParams(_runtime, &mxnParams)) != 0) {
    LOG(LOG_ERROR, "runtimeSetMxNParams() failed: %d", res);
    return res;
  }

  return 0;
}

void* threadInput(void* _arg) {
  int res = 0;
  intptr_t exit_code = 0;
  Runtime* runtime = (Runtime*) _arg;
  RCInput* rc;

  if (runtime == NULL) {
    exit_code = EINVAL;
    goto exit;
  }

  if ((rc = runtimeModRCInput(runtime)) == NULL) {
    exit_code = EINVAL;
    goto exit;
  }

  if ((res = rcInputOpen(rc, runtimeCfgRCInput(runtime))) != 0) {
    LOG(LOG_ERROR, "rcInputOpen() failed: %d", res);
    exit_code = res;
    goto exit;
  }

  if ((res = rcInputStart(rc)) != 0) {
    LOG(LOG_ERROR, "rcInputStart() failed: %d", res);
    exit_code = res;
    goto exit_rc_close;
  }

  while (!runtimeGetTerminate(runtime)) {
    if ((res = threadInputSelectLoop(runtime, rc)) != 0) {
      LOG(LOG_ERROR, "threadInputSelectLoop() failed: %d", res);
      exit_code = res;
      goto exit_rc_stop;
    }
  }

exit_rc_stop:
  if ((res = rcInputStop(rc)) != 0)
    LOG(LOG_ERROR, "rcInputStop() failed: %d", res);

exit_rc_close:
  if ((res = rcInputClose(rc)) != 0)
    LOG(LOG_ERROR, "rcInputClose() failed: %d", res);

exit:
  runtimeSetTerminate(runtime);
  return (void*) exit_code;
}

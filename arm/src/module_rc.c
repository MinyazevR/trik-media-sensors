#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "trik/sensors/log.h"
#include "trik/sensors/module_rc.h"

static int do_openFifoInput(RCInput* _rc, const char* _fifoInputName) {
  int res;
  if (_rc == NULL)
    return EINVAL;

  if (_rc->m_fifoInputName != NULL)
    free(_rc->m_fifoInputName);
  _rc->m_fifoInputName = NULL;

  if (_fifoInputName == NULL) {
    _rc->m_fifoInputFd = -1;
    return 0;
  }

  if (mkfifo(_fifoInputName, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP) < 0) {
    res = errno;
    if (res != EEXIST)
      LOG(LOG_ERROR, "mkfifo(%s) failed, continuing: %d", _fifoInputName, res);
  }

  _rc->m_fifoInputFd = open(_fifoInputName, O_RDWR | O_NONBLOCK);
  if (_rc->m_fifoInputFd < 0) {
    res = errno;
    LOG(LOG_ERROR, "open(%s) failed: %d", _fifoInputName, errno);
    _rc->m_fifoInputFd = -1;
    unlink(_fifoInputName);
    return res;
  }
  _rc->m_fifoInputName = strdup(_fifoInputName);
  _rc->m_fifoInputReadBufferUsed = 0;

  return 0;
}

static int do_closeFifoInput(RCInput* _rc) {
  int res;
  int exit_code = 0;

  if (_rc == NULL)
    return EINVAL;

  if (_rc->m_fifoInputFd != -1 && close(_rc->m_fifoInputFd) != 0) {
    res = errno;
    LOG(LOG_ERROR, "close() failed: %d", res);
    exit_code = res;
  }
  _rc->m_fifoInputFd = -1;

  if (_rc->m_fifoInputName != NULL) {
    if (unlink(_rc->m_fifoInputName) != 0) {
      res = errno;
      if (res != EBUSY) {
        LOG(LOG_ERROR, "unlink(%s) failed: %d", _rc->m_fifoInputName, res);
        exit_code = res;
      }
    }
    free(_rc->m_fifoInputName);
    _rc->m_fifoInputName = NULL;
  }

  return exit_code;
}

static int do_reopenFifoInput(RCInput* _rc) {
  int res;
  if (_rc == NULL)
    return EINVAL;

  if (_rc->m_fifoInputFd != -1 && close(_rc->m_fifoInputFd) != 0) {
    res = errno;
    LOG(LOG_ERROR, "close() failed: %d", res);
    _rc->m_fifoInputFd = -1;
    return res;
  }

  _rc->m_fifoInputFd = open(_rc->m_fifoInputName, O_RDONLY | O_NONBLOCK);
  if (_rc->m_fifoInputFd < 0) {
    res = errno;
    LOG(LOG_ERROR, "open(%s) failed: %d", _rc->m_fifoInputName, errno);
    _rc->m_fifoInputFd = -1;
    return res;
  }

  return 0;
}

static int do_openFifoOutput(RCInput* _rc, const char* _fifoOutputName) {
  int res;
  if (_rc == NULL)
    return EINVAL;

  if (_rc->m_fifoOutputName != NULL)
    free(_rc->m_fifoOutputName);
  _rc->m_fifoOutputName = NULL;

  if (_fifoOutputName == NULL) {
    _rc->m_fifoOutputFd = -1;
    return 0;
  }

  if (mkfifo(_fifoOutputName, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP) < 0) {
    res = errno;
    if (res != EEXIST)
      LOG(LOG_ERROR, "mkfifo(%s) failed, continuing: %d", _fifoOutputName, res);
  }

  int fifoOutputRdFd = open(_fifoOutputName, O_RDONLY | O_NONBLOCK);
  if (fifoOutputRdFd < 0) {
    res = errno;
    LOG(LOG_ERROR, "open(%s, RD_ONLY side) failed: %d", _fifoOutputName, errno);
    unlink(_fifoOutputName);
    return res;
  }

  _rc->m_fifoOutputFd = open(_fifoOutputName, O_WRONLY | O_NONBLOCK);
  if (_rc->m_fifoOutputFd < 0) {
    res = errno;
    LOG(LOG_ERROR, "open(%s, WR_ONLY side) failed: %d", _fifoOutputName, errno);
    close(fifoOutputRdFd);
    _rc->m_fifoOutputFd = -1;
    unlink(_fifoOutputName);
    return res;
  }

  if (close(fifoOutputRdFd) != 0) {
    res = errno;
    LOG(LOG_ERROR, "close(RD_ONLY side) failed: %d", res);
  }

  _rc->m_fifoOutputName = strdup(_fifoOutputName);

  return 0;
}

static int do_closeFifoOutput(RCInput* _rc) {
  int res;
  int exit_code = 0;

  if (_rc == NULL)
    return EINVAL;

  if (_rc->m_fifoOutputFd != -1 && close(_rc->m_fifoOutputFd) != 0) {
    res = errno;
    LOG(LOG_ERROR, "close() failed: %d", res);
    exit_code = res;
  }
  _rc->m_fifoOutputFd = -1;

  if (_rc->m_fifoOutputName != NULL) {
    if (unlink(_rc->m_fifoOutputName) != 0) {
      res = errno;
      if (res != EBUSY) {
        LOG(LOG_ERROR, "unlink(%s) failed: %d", _rc->m_fifoOutputName, res);
        exit_code = res;
      }
    }
    free(_rc->m_fifoOutputName);
    _rc->m_fifoOutputName = NULL;
  }

  return exit_code;
}

static int do_startTargetDetectParams(RCInput* _rc) {
  if (_rc == NULL)
    return EINVAL;

  _rc->m_targetDetectParamsUpdated = false;
  _rc->m_targetDetectCommandUpdated = false;

  return 0;
}

static int do_stopTargetDetectParams(RCInput* _rc) {
  if (_rc == NULL)
    return EINVAL;

  _rc->m_targetDetectParamsUpdated = false;
  _rc->m_targetDetectCommandUpdated = false;

  return 0;
}

static int do_readFifoInput(RCInput* _rc) {
  int res;

  if (_rc == NULL)
    return EINVAL;

  if (_rc->m_fifoInputFd == -1)
    return ENOTCONN;

  if (_rc->m_fifoInputReadBuffer == NULL || _rc->m_fifoInputReadBufferSize == 0)
    return EBUSY;

  if (_rc->m_fifoInputReadBufferUsed >= _rc->m_fifoInputReadBufferSize - 1) {
    LOG(LOG_ERROR, "Input fifo overflow, truncated");
    _rc->m_fifoInputReadBufferUsed = 0;
  }

  const size_t available = _rc->m_fifoInputReadBufferSize - _rc->m_fifoInputReadBufferUsed - 1; // reserve space for appended trailing zero
  const ssize_t read_res = read(_rc->m_fifoInputFd, _rc->m_fifoInputReadBuffer + _rc->m_fifoInputReadBufferUsed, available);
  if (read_res <= 0) {
    if (read_res == 0)
      LOG(LOG_ERROR, "read(%d, %zu) eof", _rc->m_fifoInputFd, available);
    else {
      res = errno;
      LOG(LOG_ERROR, "read(%d, %zu) failed: %d", _rc->m_fifoInputFd, available, res);
    }

    if ((res = do_reopenFifoInput(_rc)) != 0) {
      LOG(LOG_ERROR, "reopen fifo input failed: %d", res);
      return res;
    }

    LOG(LOG_ERROR, "reopened input fifo");

    return 0;
  }

  _rc->m_fifoInputReadBufferUsed += read_res;
  _rc->m_fifoInputReadBuffer[_rc->m_fifoInputReadBufferUsed] = '\0';

  char* parseAt = _rc->m_fifoInputReadBuffer;
  char* parseTill;
  while ((parseTill = strchr(parseAt, '\n')) != NULL) {
    *parseTill = '\0';

    if (strncmp(parseAt, "detect", strlen("detect")) == 0) {
      _rc->m_targetDetectCommand = 1;
      _rc->m_targetDetectCommandUpdated = true;
    } else if (strncmp(parseAt, "hsv ", strlen("hsv ")) == 0) {
      int hue, hueTol, sat, satTol, val, valTol;
      parseAt += strlen("hsv ");

      if ((sscanf(parseAt, "%d %d %d %d %d %d", &hue, &hueTol, &sat, &satTol, &val, &valTol)) != 6)
        LOG(LOG_ERROR, "Cannot parse hsv command, args '%s'", parseAt);
      else {
        _rc->m_targetDetectHue = hue;
        _rc->m_targetDetectHueTolerance = hueTol;
        _rc->m_targetDetectSat = sat;
        _rc->m_targetDetectSatTolerance = satTol;
        _rc->m_targetDetectVal = val;
        _rc->m_targetDetectValTolerance = valTol;
        _rc->m_targetDetectParamsUpdated = true;
      }
    } else if (strncmp(parseAt, "mxn ", strlen("mxn ")) == 0) {
      int m, n;
      parseAt += strlen("mxn ");

      if ((sscanf(parseAt, "%d %d", &m, &n)) != 2)
        LOG(LOG_ERROR, "Cannot parse mxn command, args '%s'", parseAt);
      else
      {
        _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParams.m_m    = m < COLORS_WIDTHM_MAX ? m : COLORS_WIDTHM_MAX;
        _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParams.m_n    = n < COLORS_HEIGHTN_MAX ? n : COLORS_HEIGHTN_MAX;;
        _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParamsUpdated = true;
      }
    } else if (strncmp(parseAt, "video_out ", strlen("video_out ")) == 0) {
      bool videoOutEnable;
      parseAt += strlen("video_out ");

      if ((sscanf(parseAt, "%d", &videoOutEnable)) != 1)
        LOG(LOG_ERROR, "Cannot parse video_out command, args '%s'", parseAt);
      else {
        _rc->m_videoOutEnable = videoOutEnable;
        _rc->m_videoOutParamsUpdated = true;
      }
    } else
      LOG(LOG_ERROR, "Unknown command '%s'", parseAt);

    parseAt = parseTill + 1;
  }

  _rc->m_fifoInputReadBufferUsed -= (parseAt - _rc->m_fifoInputReadBuffer);
  memmove(_rc->m_fifoInputReadBuffer, parseAt, _rc->m_fifoInputReadBufferUsed);

  return 0;
}

int rcInputInit(void) {
  return 0;
}

int rcInputFini() { return 0; }

int rcInputOpen(RCInput* _rc, const RCConfig* _config) {
  int res = 0;

  if (_rc == NULL)
    return EINVAL;
  if (_rc->m_fifoInputFd != -1 || _rc->m_fifoOutputFd != -1)
    return EALREADY;

  if ((res = do_openFifoInput(_rc, _config->m_fifoInput)) != 0)
    return res;

  if ((res = do_openFifoOutput(_rc, _config->m_fifoOutput)) != 0) {
    do_closeFifoInput(_rc);
    return res;
  }

  _rc->m_fifoInputReadBufferSize = 1000;
  _rc->m_fifoInputReadBufferUsed = 0;
  _rc->m_fifoInputReadBuffer = malloc(_rc->m_fifoInputReadBufferSize);

  _rc->m_videoOutEnable = _config->m_videoOutEnable;
  _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParams = _config->m_extraParams.m_mxnParams;

  return 0;
}

int rcInputClose(RCInput* _rc) {
  if (_rc == NULL)
    return EINVAL;
  if (_rc->m_fifoInputFd == -1 && _rc->m_fifoOutputFd == -1)
    return EALREADY;

  if (_rc->m_fifoInputReadBuffer)
    free(_rc->m_fifoInputReadBuffer);
  _rc->m_fifoInputReadBuffer = NULL;
  _rc->m_fifoInputReadBufferSize = 0;

  do_closeFifoOutput(_rc);
  do_closeFifoInput(_rc);

  return 0;
}

int rcInputStart(RCInput* _rc) {
  int res;

  if (_rc == NULL)
    return EINVAL;
  if (_rc->m_fifoInputFd == -1 && _rc->m_fifoOutputFd == -1)
    return ENOTCONN;

  if ((res = do_startTargetDetectParams(_rc)) != 0)
    return res;

  return 0;
}

int rcInputStop(RCInput* _rc) {
  if (_rc == NULL)
    return EINVAL;
  if (_rc->m_fifoInputFd == -1 && _rc->m_fifoOutputFd == -1)
    return ENOTCONN;

  do_stopTargetDetectParams(_rc);

  return 0;
}

int rcInputReadFifoInput(RCInput* _rc) {
  int res;

  if (_rc == NULL)
    return EINVAL;

  if ((res = do_readFifoInput(_rc)) != 0)
    return res;

  return 0;
}

int rcInputGetTargetDetectParams(RCInput* _rc, trik_cv_algorithm_in_args* _targetDetectParams) {
  if (_rc == NULL || _targetDetectParams == NULL)
    return EINVAL;

  if (!_rc->m_targetDetectParamsUpdated)
    return ENODATA;

  _rc->m_targetDetectParamsUpdated = false;
  _targetDetectParams->detect_hue_from = _rc->m_targetDetectHue;
  _targetDetectParams->detect_hue_to = _rc->m_targetDetectHueTolerance;
  _targetDetectParams->detect_sat_from = _rc->m_targetDetectSat;
  _targetDetectParams->detect_sat_to = _rc->m_targetDetectSatTolerance;
  _targetDetectParams->detect_val_from = _rc->m_targetDetectVal;
  _targetDetectParams->detect_val_to = _rc->m_targetDetectValTolerance;

  return 0;
}

int rcInputGetVideoOutParams(RCInput* _rc, bool* _videoOutEnable) {
  if (_rc == NULL || _videoOutEnable == NULL)
    return EINVAL;

  _rc->m_videoOutParamsUpdated = false;
  *_videoOutEnable = _rc->m_videoOutEnable;

  return 0;
}

int rcInputGetMxNParams(RCInput* _rc, MxnParams* mxnParams) {
  if (_rc == NULL || mxnParams == NULL)
    return EINVAL;

  _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParamsUpdated = false;
  *mxnParams = _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParams;

  return 0;
}

int rcInputGetTargetDetectCommand(RCInput* _rc, TargetDetectCommand* _targetDetectCommand) {
  if (_rc == NULL || _targetDetectCommand == NULL)
    return EINVAL;

  if (!_rc->m_targetDetectCommandUpdated)
    return ENODATA;

  _rc->m_targetDetectCommandUpdated = false;
  _targetDetectCommand->m_cmd = _rc->m_targetDetectCommand;

  return 0;
}

#warning TODO code below if unsafe since it is used from another thread; consider reworking

int rcInputUnsafeReportTargetLocation(RCInput* _rc, const TargetLocation* _targetLocation) {
  if (_rc == NULL || _targetLocation == NULL)
    return EINVAL;

  if (_rc->m_fifoOutputFd != -1)
    dprintf(_rc->m_fifoOutputFd, "loc: %d %d %d\n", _targetLocation->x, _targetLocation->y, _targetLocation->size);

  return 0;
}

#warning TODO code below if unsafe since it is used from another thread; consider reworking
int rcInputUnsafeReportTargetColors(RCInput* _rc, const TargetColors* _targetColors)
{
  if (_rc == NULL || _targetColors == NULL)
    return EINVAL;

  if (_rc->m_fifoOutputFd != -1)
  {
    dprintf(_rc->m_fifoOutputFd, "color: ");
    int i = 0;
    for (i = 0; i < _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParams.m_m * _rc->m_extraRCInput.m_mxnParamsInput.m_mxnParams.m_n; i++)
    {
      dprintf(_rc->m_fifoOutputFd, "%d ", _targetColors->m_colors[i]);
    }

    dprintf(_rc->m_fifoOutputFd, "\n");
  }

  return 0;
}

#warning TODO code below if unsafe since it is used from another thread; consider reworking
int rcInputUnsafeReportTargetDetectParams(RCInput* _rc, const trik_cv_algorithm_out_args* _targetDetectParams) {
  if (_rc == NULL || _targetDetectParams == NULL)
    return EINVAL;

  if (_rc->m_fifoOutputFd != -1)
    dprintf(_rc->m_fifoOutputFd, "hsv: %d %d %d %d %d %d\n", _targetDetectParams->detect_hue_from, _targetDetectParams->detect_hue_to,
      _targetDetectParams->detect_sat_from, _targetDetectParams->detect_sat_to, _targetDetectParams->detect_val_from, _targetDetectParams->detect_val_to);

  if (_rc->m_fifoInputFd != -1) {
    dprintf(_rc->m_fifoInputFd, "hsv %d %d %d %d %d %d\n", _targetDetectParams->detect_hue_from, _targetDetectParams->detect_hue_to,
           _targetDetectParams->detect_sat_from, _targetDetectParams->detect_sat_to, _targetDetectParams->detect_val_from, _targetDetectParams->detect_val_to);
  }
  return 0;
}

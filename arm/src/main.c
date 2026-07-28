#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ti/ipc/Std.h>

#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/transports/TransportRpmsg.h>

#include <ti/ipc/MultiProc.h>

#include "trik/sensors/runtime.h"
#include <errno.h>
#include <signal.h>
#include <sysexits.h>
#include <trik/sensors/arm_server.h>
#include <trik/sensors/cmd.h>
#include <trik/sensors/cv_algorithm.h>
#include <trik/sensors/log.h>
#include <trik/sensors/msg.h>

log_level_t g_log_level = LOG_ERROR;

static sig_atomic_t s_signalTerminate = false;

static void sigterm_action(int _signal, siginfo_t* _siginfo, void* _context) {
  (void) _signal;
  (void) _siginfo;
  (void) _context;
  s_signalTerminate = true;
}

static int sigactions_setup() {
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  action.sa_sigaction = &sigterm_action;
  action.sa_flags = SA_SIGINFO | SA_RESTART;

  if (sigaction(SIGTERM, &action, NULL) != 0) {
    LOG(LOG_ERROR, "sigaction(SIGTERM) failed: %d", errno);
    return -1;
  }
  if (sigaction(SIGINT, &action, NULL) != 0) {
    LOG(LOG_ERROR, "sigaction(SIGINT) failed: %d", errno);
    return -1;
  }

  signal(SIGPIPE, SIG_IGN);
  return 0;
}

int main(int _argc, char* const _argv[]) {
  Runtime runtime;

  int res = 0;
  int exit_code = EX_OK;
  const char* arg0 = _argv[0];

  runtimeReset(&runtime);
  if (!runtimeParseArgs(&runtime, _argc, _argv)) {
    runtimeArgsHelpMessage(&runtime, arg0);
    exit_code = EX_USAGE;
    goto exit;
  }

  if ((res = runtimeInit(&runtime)) != 0) {
    LOG(LOG_ERROR, "runtimeInit() failed: %d", res);
    exit_code = EX_SOFTWARE;
    goto exit;
  }

  if ((res = Ipc_transportConfig(&TransportRpmsg_Factory)) != 0) {
    LOG(LOG_ERROR, "Ipc_transportConfig failed: status = %d", res);
    exit_code = EX_SOFTWARE;
    goto exit;
  }

  if ((res = Ipc_start()) < 0) {
    LOG(LOG_ERROR, "Ipc_start failed: status = %d", res);
    exit_code = EX_SOFTWARE;
    goto exit;
  }

  uint16_t rproc_id;

  rproc_id = MultiProc_getId("DSP");

  if ((res = trik_init_arm_server(rproc_id)) < 0) {
    LOG(LOG_ERROR, "main(): failed to initialize trik arm server: %d", res);
    exit_code = EX_SOFTWARE;
    goto exit_ipc_stop;
  }

  if ((res = sigactions_setup()) != 0) {
    LOG(LOG_ERROR, "sigactions_setup failed: %d", res);
    exit_code = EX_SOFTWARE;
    goto exit_ipc_stop;
  }

  if ((res = runtimeStart(&runtime)) != 0) {
    LOG(LOG_ERROR, "runtimeStart failed: %d", res);
    exit_code = EX_SOFTWARE;
    goto exit_ipc_stop;
  }

  while (!s_signalTerminate && !runtimeGetTerminate(&runtime)) {
    sleep(1);
  }

exit_runtime_stop:
  if ((res = runtimeStop(&runtime)) != 0) {
    LOG(LOG_ERROR, "runtimeStop() failed: %d", res);
    exit_code = EX_SOFTWARE;
  }
exit_fini:
  if ((res = runtimeFini(&runtime)) != 0) {
    LOG(LOG_ERROR, "runtimeStop() failed: %d", res);
    exit_code = EX_SOFTWARE;
  }

exit_ipc_stop:
  if ((res = Ipc_stop()) < 0) {
    LOG(LOG_ERROR, "Ipc_stop failed: status = %d", res);
    exit_code = EX_SOFTWARE;
  }

exit:
  return exit_code;
}

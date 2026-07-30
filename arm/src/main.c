#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sysexits.h>
#include <unistd.h>

#include <ti/ipc/Std.h>

#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/transports/TransportRpmsg.h>

#include "trik/sensors/app.h"
#include "trik/sensors/arm_server.h"
#include "trik/sensors/input_loop.h"
#include "trik/sensors/video_loop.h"

static volatile sig_atomic_t s_sigterm;

static void sigterm_handler(int sig, siginfo_t* info, void* ctx)
{
  (void)sig; (void)info; (void)ctx;
  s_sigterm = 1;
}

static int setup_signals(void)
{
  struct sigaction sa = { .sa_sigaction = sigterm_handler,
                          .sa_flags = SA_SIGINFO | SA_RESTART };
  if (sigaction(SIGTERM, &sa, NULL) || sigaction(SIGINT, &sa, NULL)) {
    fprintf(stderr, "sigaction failed: %d\n", errno);
    return -1;
  }
  signal(SIGPIPE, SIG_IGN);
  return 0;
}

int main(int argc, char* argv[])
{
  struct app app;
  memset(&app, 0, sizeof(app));

  if (app_config_init(&app.config, argc, argv) != 0)
    return EX_USAGE;

  if (!app.config.ctrl_fifo_path) {
    fprintf(stderr, "--ctrl-fifo is required\n");
    return EX_USAGE;
  }

  /* Management FIFO */
  if (mkfifo(app.config.ctrl_fifo_path, S_IRUSR | S_IWUSR) < 0 && errno != EEXIST)
    fprintf(stderr, "mkfifo(%s): %d\n", app.config.ctrl_fifo_path, errno);

  app.ctrl_fd = open(app.config.ctrl_fifo_path, O_RDWR | O_NONBLOCK);
  if (app.ctrl_fd < 0) {
    fprintf(stderr, "open(%s): %d\n", app.config.ctrl_fifo_path, errno);
    return EX_SOFTWARE;
  }

  /* IPC with DSP */
  if (Ipc_transportConfig(&TransportRpmsg_Factory) || Ipc_start() < 0) {
    fprintf(stderr, "IPC init failed\n");
    return EX_SOFTWARE;
  }

  uint16_t rproc_id = MultiProc_getId("DSP");
  if (trik_init_arm_server(rproc_id) < 0) {
    fprintf(stderr, "DSP init failed\n");
    return EX_SOFTWARE;
  }

  if (setup_signals() != 0)
    return EX_SOFTWARE;

  /* Start worker threads */
  if (pthread_create(&app.input_thread, NULL,
                     (void* (*)(void*))input_loop_run, &app) != 0) {
    fprintf(stderr, "pthread_create(input) failed\n");
    return EX_SOFTWARE;
  }
  if (pthread_create(&app.video_thread, NULL,
                     (void* (*)(void*))video_loop_run, &app) != 0) {
    fprintf(stderr, "pthread_create(video) failed\n");
    app.terminate = true;
    pthread_join(app.input_thread, NULL);
    return EX_SOFTWARE;
  }

  /* Wait for shutdown */
  while (!s_sigterm && !app.terminate)
    sleep(1);

  app.terminate = true;

  pthread_join(app.video_thread, NULL);
  pthread_join(app.input_thread, NULL);

  trik_destroy_arm_server();
  Ipc_stop();

  if (app.ctrl_fd != -1)
    close(app.ctrl_fd);

  return EX_OK;
}

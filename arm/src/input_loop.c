#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <unistd.h>

#include "trik/sensors/app.h"
#include "trik/sensors/channel.h"
#include "trik/sensors/config.h"
#include "trik/sensors/input_loop.h"
#include "trik/sensors/schedule.h"

/* split "a b c\n" into argv[0..2]. Returns argc. */
static int tokenize(char* p, char* argv[], int max_args)
{
  int argc = 0;
  while (argc < max_args) {
    while (*p == ' ' || *p == '\n')  /* skip whitespace */
      *p++ = '\0';
    if (!*p)
      break;
    argv[argc++] = p;
    while (*p && *p != ' ' && *p != '\n')
      p++;
  }
  return argc;
}

static enum trik_cv_algorithm algo_from_string(const char* s)
{
  if (strcmp(s, "line_sensor") == 0)        return TRIK_CV_ALGORITHM_LINE_SENSOR;
  if (strcmp(s, "motion_sensor") == 0)      return TRIK_CV_ALGORITHM_MOTION_SENSOR;
  if (strcmp(s, "object_sensor") == 0)      return TRIK_CV_ALGORITHM_OBJECT_SENSOR;
  if (strcmp(s, "edge_line_sensor") == 0)   return TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR;
  if (strcmp(s, "mxn_sensor") == 0)         return TRIK_CV_ALGORITHM_MXN_SENSOR;
  return TRIK_CV_ALGORITHM_NONE;
}

static void handle_add_camera(struct app* app, int argc, char* argv[])
{
  /*
   * add_camera <id> <path> [<width> <height> <format>]
   *   add_camera 0 /dev/video0
   *   add_camera 0 /dev/video0 640 480 yuv422
   */
  int id = atoi(argv[0]);
  if (id < 0 || id >= MAX_CAMERAS) {
    fprintf(stderr, "add_camera: bad id %d\n", id);
    return;
  }

  struct camera_config cfg;
  if (camera_config_parse(&cfg, argc - 1, argv + 1) != 0) {
    fprintf(stderr, "add_camera: parse failed\n");
    return;
  }

  struct camera* cam = &app->cams[id];
  camera_remove(cam);
  camera_init(cam, id);
  if (camera_add(cam, &cfg) != 0) {
    fprintf(stderr, "add_camera: failed to open %s\n", cfg.path);
    return;
  }

  fprintf(stderr, "add_camera: id=%d path=%s %zux%zu\n",
          id, cfg.path, cfg.width, cfg.height);
}

static void handle_bind(struct app* app, int argc, char* argv[])
{
  /*
   * bind <cam_id> <algo> <fifo_in> <fifo_out>
   *   bind 0 line_sensor /run/line.in /run/line.out
   */
  if (argc < 4) {
    fprintf(stderr, "bind: need 4 args, got %d\n", argc);
    return;
  }

  int cam_id = atoi(argv[0]);
  if (cam_id < 0 || cam_id >= MAX_CAMERAS) {
    fprintf(stderr, "bind: bad cam_id %d\n", cam_id);
    return;
  }

  enum trik_cv_algorithm algo = algo_from_string(argv[1]);
  if (algo == TRIK_CV_ALGORITHM_NONE) {
    fprintf(stderr, "bind: unknown algo '%s'\n", argv[1]);
    return;
  }

  int idx = cam_id * TRIK_CV_ALGORITHM_COUNT + algo;
  struct sensor_channel* ch = &app->channels[idx];

  /* Rebind: close old FIFOs if already active. */
  if (ch->bound) {
    close(ch->fd_in);
    close(ch->fd_out);
    free(ch->read_buf);
  }

  memset(ch, 0, sizeof(*ch));
  ch->cam_id = cam_id;
  ch->algo = algo;
  ch->priority = 1;
  ch->fd_in = -1;
  ch->fd_out = -1;
  pthread_mutex_init(&ch->mutex, NULL);

  if (mkfifo(argv[2], S_IRUSR | S_IWUSR) < 0 && errno != EEXIST)
    fprintf(stderr, "bind: mkfifo(%s): %d\n", argv[2], errno);

  ch->fd_in = open(argv[2], O_RDWR | O_NONBLOCK);
  if (ch->fd_in < 0) {
    fprintf(stderr, "bind: open(%s): %d\n", argv[2], errno);
    return;
  }

  if (mkfifo(argv[3], S_IRUSR | S_IWUSR) < 0 && errno != EEXIST)
    fprintf(stderr, "bind: mkfifo(%s): %d\n", argv[3], errno);

  ch->fd_out = open(argv[3], O_RDWR | O_NONBLOCK);
  if (ch->fd_out < 0) {
    fprintf(stderr, "bind: open(%s): %d\n", argv[3], errno);
    close(ch->fd_in);
    return;
  }

  ch->read_buf = malloc(1024);
  if (!ch->read_buf) {
    fprintf(stderr, "bind: malloc failed\n");
    close(ch->fd_out);
    close(ch->fd_in);
    return;
  }
  ch->read_used = 0;
  ch->read_size = 1024;
  ch->bound = true;
  schedule_rebuild(app);

  /* Ensure camera is streaming */
  if (app->cams[cam_id].ready && !app->cams[cam_id].streaming) {
    trik_v4l2_start(&app->cams[cam_id].v4l2);
    app->cams[cam_id].streaming = true;
  }

  fprintf(stderr, "bind: cam=%d algo=%s fi=%s fo=%s\n",
          cam_id, argv[1], argv[2], argv[3]);
}

static void handle_unbind(struct app* app, int argc, char* argv[])
{
  /*
   * unbind <cam_id> <algo>
   *   unbind 0 line_sensor
   */
  if (argc < 2) {
    fprintf(stderr, "unbind: need 2 args\n");
    return;
  }

  int cam_id = atoi(argv[0]);
  enum trik_cv_algorithm algo = algo_from_string(argv[1]);
  if (algo == TRIK_CV_ALGORITHM_NONE) {
    fprintf(stderr, "unbind: unknown algo '%s'\n", argv[1]);
    return;
  }

  struct sensor_channel* ch = &app->channels[cam_id * TRIK_CV_ALGORITHM_COUNT + algo];
  if (!ch->bound) {
    fprintf(stderr, "unbind: not bound\n");
    return;
  }

  close(ch->fd_in);
  close(ch->fd_out);
  free(ch->read_buf);
  ch->fd_in = -1;
  ch->fd_out = -1;
  pthread_mutex_destroy(&ch->mutex);
  ch->bound = false;
  schedule_rebuild(app);

  /* Stop camera stream if this was the last sensor on it */
  bool cam_has_bound = false;
  for (int i = 0; i < MAX_CHANNELS; i++)
    if (app->channels[i].bound && app->channels[i].cam_id == cam_id)
      cam_has_bound = true;
  if (!cam_has_bound && app->cams[cam_id].ready) {
    trik_v4l2_stop(&app->cams[cam_id].v4l2);
    app->cams[cam_id].streaming = false;
  }

  fprintf(stderr, "unbind: cam=%d algo=%s\n", cam_id, argv[1]);
}

static void handle_ctrl(struct app* app)
{
  char buf[256];
  int n = read(app->ctrl_fd, buf, sizeof(buf) - 1);
  if (n <= 0)
    return;
  buf[n] = '\0';

  char* argv[8];
  int argc = tokenize(buf, argv, 8);
  if (argc == 0)
    return;

  if (strcmp(argv[0], "add_camera") == 0) {
    handle_add_camera(app, argc - 1, argv + 1);
  } else if (strcmp(argv[0], "bind") == 0) {
    handle_bind(app, argc - 1, argv + 1);
  } else if (strcmp(argv[0], "unbind") == 0) {
    handle_unbind(app, argc - 1, argv + 1);
  } else if (strcmp(argv[0], "shutdown") == 0) {
    for (int i = 0; i < MAX_CHANNELS; i++) {
      if (app->channels[i].bound) {
        close(app->channels[i].fd_in);
        close(app->channels[i].fd_out);
        free(app->channels[i].read_buf);
        app->channels[i].bound = false;
      }
    }
    for (int i = 0; i < MAX_CAMERAS; i++)
      if (app->cams[i].ready && app->cams[i].streaming) {
        trik_v4l2_stop(&app->cams[i].v4l2);
        app->cams[i].streaming = false;
      }
    app->terminate = true;
    fprintf(stderr, "ctrl-fifo: shutdown\n");
  } else {
    fprintf(stderr, "ctrl-fifo: unknown command '%s'\n", argv[0]);
  }
}

void input_loop_run(struct app* app)
{
  if (!app)
    return;

  struct timespec timeout = { .tv_sec = 1, .tv_nsec = 0 };

  while (!app->terminate) {
    fd_set fds;
    int max_fd = 0;

    FD_ZERO(&fds);
    FD_SET(app->ctrl_fd, &fds);
    max_fd = app->ctrl_fd;

    for (int i = 0; i < MAX_CHANNELS; i++) {
      if (app->channels[i].bound) {
        FD_SET(app->channels[i].fd_in, &fds);
        if (app->channels[i].fd_in > max_fd)
          max_fd = app->channels[i].fd_in;
      }
    }

    int res = pselect(max_fd + 1, &fds, NULL, NULL, &timeout, NULL);
    if (res < 0) {
      if (errno == EINTR)
        continue;
      fprintf(stderr, "pselect() failed: %d\n", errno);
      break;
    }

    if (FD_ISSET(app->ctrl_fd, &fds))
      handle_ctrl(app);

    for (int i = 0; i < MAX_CHANNELS; i++) {
      if (app->channels[i].bound
          && FD_ISSET(app->channels[i].fd_in, &fds))
        channel_parse(&app->channels[i]);
    }
  }
}

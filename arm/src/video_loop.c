#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include <trik/buffer.h>

#include "trik/sensors/app.h"
#include "trik/sensors/arm_server.h"
#include "trik/sensors/channel.h"
#include "trik/sensors/schedule.h"
#include "trik/sensors/video_loop.h"

void video_loop_run(struct app* app)
{
  if (!app)
    return;

  /* DSP: map shared memory buffers */
  if (trik_dsp_init_buffer(&app->dsp_in, &app->dsp_out) < 0) {
    fprintf(stderr, "video_loop: DSP init failed\n");
    return;
  }

  /* Display */
  if (app->config.fb_path)
    trik_fb_open(&app->fb, app->config.fb_path);

  /* Register algorithms on DSP, start cameras */
  for (int i = 0; i < MAX_CAMERAS; i++) {
    if (!app->cams[i].ready)
      continue;
    trik_v4l2_start(&app->cams[i].v4l2);
    app->cams[i].streaming = true;
  }

  /* Build initial schedule */
  schedule_rebuild(app);

  struct timespec timeout = { .tv_sec = 1, .tv_nsec = 0 };
  int current_algo = -1;

  while (!app->terminate) {
    fd_set fds;
    int max_fd = 0;

    FD_ZERO(&fds);
    for (int i = 0; i < MAX_CAMERAS; i++) {
      if (app->cams[i].ready && app->cams[i].streaming) {
        FD_SET(app->cams[i].v4l2.fd, &fds);
        if (app->cams[i].v4l2.fd > max_fd)
          max_fd = app->cams[i].v4l2.fd;
      }
    }

    int res = pselect(max_fd + 1, &fds, NULL, NULL, &timeout, NULL);
    if (res < 0) {
      if (errno == EINTR)
        continue;
      fprintf(stderr, "video_loop: pselect failed: %d\n", errno);
      break;
    }

    struct schedule_slot* s = schedule_next(app, &fds);
    if (!s)
      continue;

    struct camera* cam = &app->cams[s->cam_id];
    struct sensor_channel* ch = &app->channels[s->ch_idx];

    const void* ptr;
    size_t sz, idx;
    if (trik_v4l2_get_frame(&cam->v4l2, &ptr, &sz, &idx) != 0)
      continue;

    /* Re-register algorithm on DSP if type changed */
    if (ch->algo != current_algo) {
      trik_dsp_register_algo(ch->algo,
                             cam->v4l2.format.fmt.pix.pixelformat,
                             cam->v4l2.format.fmt.pix.bytesperline);
      current_algo = ch->algo;
    }

    /* Get latest params from input thread */
    trik_cv_algorithm_in_args params;
    channel_get_params(ch, &params);
    bool did_detect = channel_consume_detect(ch, &params);

    /* Send frame to DSP */
    memcpy(app->dsp_in.start, ptr, sz);
    trik_cv_algorithm_out_args out;
    if (trik_req_step(&out, params) < 0) {
      fprintf(stderr, "video_loop: trik_req_step failed\n");
      trik_v4l2_put_frame(&cam->v4l2, idx);
      continue;
    }

    /* Write result to controller */
    if (did_detect) {
      channel_write_hsv(ch, out.detect_hue_from, out.detect_hue_to,
                        out.detect_sat_from, out.detect_sat_to,
                        out.detect_val_from, out.detect_val_to);
    } else if (ch->algo == TRIK_CV_ALGORITHM_MXN_SENSOR) {
      channel_write_colors(ch, out.targets[0].out_target.targetColors.m_colors);
    } else {
      channel_write_loc(ch, out.targets[0].out_target.targetLocation.x,
                        out.targets[0].out_target.targetLocation.y,
                        out.targets[0].out_target.targetLocation.size);
    }

    /* Display */
    if (channel_get_video_out(ch))
      memcpy(app->fb.buf, app->dsp_out.start, app->fb.size);

    trik_v4l2_put_frame(&cam->v4l2, idx);
  }

  /* Cleanup */
  for (int i = 0; i < MAX_CAMERAS; i++) {
    if (app->cams[i].ready) {
      trik_v4l2_stop(&app->cams[i].v4l2);
      trik_v4l2_close(&app->cams[i].v4l2);
      app->cams[i].streaming = false;
      app->cams[i].ready = false;
    }
  }

  if (app->fb.buf)
    trik_fb_close(&app->fb);
}

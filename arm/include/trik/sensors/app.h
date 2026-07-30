#ifndef TRIK_SENSORS_APP_H_
#define TRIK_SENSORS_APP_H_

#include <pthread.h>
#include <stdbool.h>

#include "trik/sensors/camera.h"
#include "trik/sensors/channel.h"
#include "trik/sensors/config.h"
#include "trik/sensors/cv_algorithm.h"
#include "trik/sensors/module_fb.h"

#include <trik/buffer.h>

/*
 * Application root — owns cameras, sensor channels, DSP buffers and display.
 */
#define MAX_CAMERAS   2
#define MAX_CHANNELS  (TRIK_CV_ALGORITHM_COUNT * MAX_CAMERAS)

struct app {
  struct app_config config;

  struct camera cams[MAX_CAMERAS];
  struct sensor_channel channels[MAX_CHANNELS];

  struct fb_device fb;                    /* display */
  struct buffer dsp_in, dsp_out;          /* shared memory with DSP */

  int ctrl_fd;
  volatile bool terminate;

  pthread_t input_thread;
  pthread_t video_thread;
};

#endif

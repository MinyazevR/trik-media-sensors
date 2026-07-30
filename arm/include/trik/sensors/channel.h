#ifndef TRIK_SENSORS_CHANNEL_H_
#define TRIK_SENSORS_CHANNEL_H_

#include <pthread.h>
#include <stdbool.h>

#include "trik/sensors/cv_algorithm.h"
#include "trik/sensors/cv_algorithm_args.h"

/*
 * One sensor bound to a camera via the bind command.
 * Owns FIFO descriptors, command parsing buffer and runtime parameters
 * for a single CV algorithm (HSV range, video_out toggle, MxN matrix, detect flag).
 */
struct sensor_channel {
  int cam_id;
  enum trik_cv_algorithm algo;
  bool bound;

  int priority;                             /* scheduling weight, default 1 */
  pthread_mutex_t mutex;                  /* protects params, video_out, detect_cmd */

  int fd_in;                              /* sensor ← controller writes commands */
  int fd_out;                             /* sensor → controller reads results */

  char* read_buf;                         /* raw fifo input buffer */
  size_t read_used;
  size_t read_size;

  trik_cv_algorithm_in_args params;       /* HSV range + M×N grid + auto-detect */
  bool video_out;                         /* from "video_out 1" */
  int detect_cmd;                         /* from "detect" — auto-detect HSV range */
};

/*
 * Read and parse sensor commands from fd_in (hsv, detect, video_out, mxn).
 * Updates channel->params, ->video_out, ->detect_cmd under mutex.
 * Called from input thread when fd_in is readable.
 */
void channel_parse(struct sensor_channel* ch);

/*
 * Video-thread API — thread-safe access to channel state and output.
 */

/* Copy current params. */
void channel_get_params(struct sensor_channel* ch, trik_cv_algorithm_in_args* p);

/* Read video_out flag. */
bool channel_get_video_out(struct sensor_channel* ch);

/* Read detect_cmd, set auto_detect_hsv in params, clear detect_cmd.
 * Returns true if a detect was pending for this frame. */
bool channel_consume_detect(struct sensor_channel* ch, trik_cv_algorithm_in_args* p);

/* Write a target location result to fd_out. */
void channel_write_loc(struct sensor_channel* ch, int x, int y, unsigned size);

/* Write a detected HSV range to fd_out. */
void channel_write_hsv(struct sensor_channel* ch,
                       uint16_t H, uint16_t ht, uint8_t S, uint8_t st, uint8_t V, uint8_t vt);

/* Write M×N color matrix to fd_out (for mxn sensor). */
void channel_write_colors(struct sensor_channel* ch, const uint32_t* colors);

#endif

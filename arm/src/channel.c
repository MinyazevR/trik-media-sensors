#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "trik/sensors/app.h"
#include "trik/sensors/channel.h"

void channel_parse(struct sensor_channel* ch)
{
  if (!ch || !ch->bound || ch->fd_in == -1 || !ch->read_buf)
    return;

  if (ch->read_used >= ch->read_size - 1) {
    fprintf(stderr, "channel: fifo overflow, truncated\n");
    ch->read_used = 0;
  }

  size_t avail = ch->read_size - ch->read_used - 1;
  ssize_t n = read(ch->fd_in, ch->read_buf + ch->read_used, avail);
  if (n <= 0)
    return;

  ch->read_used += n;
  ch->read_buf[ch->read_used] = '\0';

  char* p = ch->read_buf;
  char* nl;
  while ((nl = strchr(p, '\n'))) {
    *nl = '\0';

    pthread_mutex_lock(&ch->mutex);

    if (strncmp(p, "hsv ", 4) == 0) {
      int H, ht, S, st, V, vt;
      if (sscanf(p + 4, "%d %d %d %d %d %d", &H, &ht, &S, &st, &V, &vt) == 6) {
        ch->params.detect_hue_from = H;
        ch->params.detect_hue_to   = ht;
        ch->params.detect_sat_from = S;
        ch->params.detect_sat_to   = st;
        ch->params.detect_val_from = V;
        ch->params.detect_val_to   = vt;
      } else {
        fprintf(stderr, "channel: bad hsv '%s'\n", p);
      }
    } else if (strcmp(p, "detect") == 0) {
      ch->detect_cmd = 1;
    } else if (strncmp(p, "video_out ", 10) == 0) {
      ch->video_out = atoi(p + 10) != 0;
    } else if (strncmp(p, "mxn ", 4) == 0) {
      int m, n;
      if (sscanf(p + 4, "%d %d", &m, &n) == 2) {
        ch->params.extra_inArgs.mxnParams.m_m = m < COLORS_WIDTHM_MAX ? m : COLORS_WIDTHM_MAX;
        ch->params.extra_inArgs.mxnParams.m_n = n < COLORS_HEIGHTN_MAX ? n : COLORS_HEIGHTN_MAX;
      } else {
        fprintf(stderr, "channel: bad mxn '%s'\n", p);
      }
    } else {
      fprintf(stderr, "channel: unknown command '%s'\n", p);
    }

    pthread_mutex_unlock(&ch->mutex);

    p = nl + 1;
  }

  ch->read_used -= (p - ch->read_buf);
  memmove(ch->read_buf, p, ch->read_used);
}

void channel_get_params(struct sensor_channel* ch, trik_cv_algorithm_in_args* p)
{
  if (!ch || !p)
    return;
  pthread_mutex_lock(&ch->mutex);
  *p = ch->params;
  pthread_mutex_unlock(&ch->mutex);
}

bool channel_get_video_out(struct sensor_channel* ch)
{
  if (!ch)
    return false;
  pthread_mutex_lock(&ch->mutex);
  bool v = ch->video_out;
  pthread_mutex_unlock(&ch->mutex);
  return v;
}

bool channel_consume_detect(struct sensor_channel* ch, trik_cv_algorithm_in_args* p)
{
  if (!ch || !p)
    return false;
  pthread_mutex_lock(&ch->mutex);
  bool pending = (ch->detect_cmd != 0);
  ch->detect_cmd = 0;
  p->auto_detect_hsv = pending;
  pthread_mutex_unlock(&ch->mutex);
  return pending;
}

#warning TODO code below if unsafe since it is used from another thread; consider reworking

void channel_write_loc(struct sensor_channel* ch, int x, int y, unsigned size)
{
  if (!ch || ch->fd_out == -1)
    return;
  dprintf(ch->fd_out, "loc: %d %d %u\n", x, y, size);
}

#warning TODO code below if unsafe since it is used from another thread; consider reworking

void channel_write_hsv(struct sensor_channel* ch,
                       uint16_t H, uint16_t ht, uint8_t S, uint8_t st, uint8_t V, uint8_t vt)
{
  if (!ch || ch->fd_out == -1)
    return;
  dprintf(ch->fd_out, "hsv: %d %d %d %d %d %d\n", H, ht, S, st, V, vt);
}

#warning TODO code below if unsafe since it is used from another thread; consider reworking

void channel_write_colors(struct sensor_channel* ch, const uint32_t* colors)
{
  if (!ch || ch->fd_out == -1)
    return;
  dprintf(ch->fd_out, "color: ");
  size_t m = ch->params.extra_inArgs.mxnParams.m_m;
  size_t n = ch->params.extra_inArgs.mxnParams.m_n;
  size_t total = (m > 0 && n > 0) ? m * n : 1;
  for (size_t i = 0; i < total; i++)
    dprintf(ch->fd_out, "%u ", colors[i]);
  dprintf(ch->fd_out, "\n");
}

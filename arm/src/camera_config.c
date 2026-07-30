#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <linux/videodev2.h>

#include "trik/sensors/config.h"

/*
 * Parse camera settings from a string like:
 *   "/dev/video0"           (default width/height, default format)
 *   "/dev/video0 640 480 yuv422"
 * Returns 0 on success, -1 on error.
 */
int camera_config_parse(struct camera_config* cfg, int argc, char* const argv[])
{
  if (!cfg || argc < 1)
    return -1;

  cfg->path   = NULL;
  cfg->width  = 320;
  cfg->height = 240;
  cfg->format = V4L2_PIX_FMT_NV16;

  cfg->path = argv[0];

  if (argc >= 3) {
    cfg->width  = (size_t)atoi(argv[1]);
    cfg->height = (size_t)atoi(argv[2]);
  }

  if (argc >= 4) {
    const char* fmt = argv[3];
    if (!strcasecmp(fmt, "rgb888"))       cfg->format = V4L2_PIX_FMT_RGB24;
    else if (!strcasecmp(fmt, "rgb565"))  cfg->format = V4L2_PIX_FMT_RGB565;
    else if (!strcasecmp(fmt, "rgb565x")) cfg->format = V4L2_PIX_FMT_RGB565X;
    else if (!strcasecmp(fmt, "yuv444"))  cfg->format = V4L2_PIX_FMT_YUV32;
    else if (!strcasecmp(fmt, "yuv422"))  cfg->format = V4L2_PIX_FMT_YUYV;
    else if (!strcasecmp(fmt, "yuv422p")) cfg->format = V4L2_PIX_FMT_YUV422P;
    else if (!strcasecmp(fmt, "nv16"))    cfg->format = V4L2_PIX_FMT_NV16;
    else {
      fprintf(stderr, "camera_config: unknown format '%s'\n", fmt);
      return -1;
    }
  }

  return 0;
}

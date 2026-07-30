#ifndef TRIK_SENSORS_CONFIG_H_
#define TRIK_SENSORS_CONFIG_H_

#include <stddef.h>
#include <stdint.h>

#define DEFAULT_FB_PATH "/dev/fb0"

/*
 * Camera configuration — device, resolution and format.
 * Comes from the management protocol (add_camera command).
 */
struct camera_config {
  const char* path;        /* /dev/videoN */
  size_t width;            /* default 320 */
  size_t height;           /* default 240 */
  uint32_t format;         /* V4L2_PIX_FMT_NV16, etc. */
};

/*
 * Application-wide daemon configuration.
 * Everything about cameras and sensors is set at runtime via the management FIFO.
 */
struct app_config {
  const char* ctrl_fifo_path; /* --ctrl-fifo: management command pipe (required) */
  const char* fb_path;        /* --fb-path: framebuffer device (default /dev/fb0) */
};

/*
 * Set defaults, parse CLI arguments, print help.
 * Returns 0 on success, -1 on parse error or --help requested.
 */
int app_config_init(struct app_config* cfg, int argc, char* const argv[]);
void app_config_help(const char* arg0);

/*
 * Parse camera config from management protocol arguments.
 *   camera_config_parse(&cfg, argc, argv)
 * args[0] = device path (required), args[1..3] = width, height, format (optional).
 * Returns 0 on success, -1 on error.
 */
int camera_config_parse(struct camera_config* cfg, int argc, char* const argv[]);

#endif

#ifndef TRIK_SENSORS_MODULE_FB_H_
#define TRIK_SENSORS_MODULE_FB_H_

#include <linux/fb.h>
#include <stdbool.h>
#include <stddef.h>

#include "trik/sensors/module_v4l2.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Opened framebuffer device state.
 * buf points directly to video memory — copy processed frames for display.
 */
struct fb_device {
  int fd;
  struct fb_fix_screeninfo fix_info;
  struct fb_var_screeninfo var_info;
  void* buf;
  size_t size;
};

/* Module init (currently a stub). */
int trik_fb_init(bool verbose);
int trik_fb_fini(void);

/* Open /dev/fbX, query screen info, mmap video memory. */
int trik_fb_open(struct fb_device* dev, const char* path);

/* Munmap and close the device. */
int trik_fb_close(struct fb_device* dev);

/* Start / stop frame output (currently stubs). */
int trik_fb_start(struct fb_device* dev);
int trik_fb_stop(struct fb_device* dev);

/* Get a pointer and size of the video memory buffer. */
int trik_fb_get_frame(struct fb_device* dev, void** ptr, size_t* sz);

/* Release the frame buffer (no-op for mmap'd memory). */
int trik_fb_put_frame(struct fb_device* dev);

/* Read back actual screen parameters into an image descriptor. */
int trik_fb_get_info(struct fb_device* dev, struct image_desc* desc);

#ifdef __cplusplus
}
#endif

#endif

/*
 * Framebuffer output — opens /dev/fbN, maps video memory via mmap,
 * and provides direct buffer access for frame display.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/videodev2.h> // pixel formats

#include "trik/sensors/module_fb.h"

static int do_trik_fb_open(struct fb_device* _fb, const char* _path) {
  int res;

  if (_fb == NULL || _path == NULL)
    return EINVAL;

  _fb->fd = open(_path, O_RDWR | O_SYNC, 0);
  if (_fb->fd < 0) {
    res = errno;
    fprintf(stderr, "open(%s) failed: %d\n", _path, res);
    _fb->fd = -1;
    return res;
  }

  return 0;
}

static int do_trik_fb_close(struct fb_device* _fb) {
  int res;
  if (_fb == NULL)
    return EINVAL;

  if (close(_fb->fd) != 0) {
    res = errno;
    fprintf(stderr, "close() failed: %d\n", res);
    return res;
  }

  return 0;
}

static int do_trik_fb_set_format(struct fb_device* _fb) {
  int res;

  if (_fb == NULL)
    return EINVAL;

  memset(&_fb->fix_info, 0, sizeof(_fb->fix_info));
  memset(&_fb->var_info, 0, sizeof(_fb->var_info));

  if (ioctl(_fb->fd, FBIOGET_FSCREENINFO, &_fb->fix_info) != 0) {
    res = errno;
    fprintf(stderr, "ioctl(FBIOGET_FSCREENINFO) failed: %d\n", res);
    return res;
  }

  if (ioctl(_fb->fd, FBIOGET_VSCREENINFO, &_fb->var_info) != 0) {
    res = errno;
    fprintf(stderr, "ioctl(FBIOGET_VSCREENINFO) failed: %d\n", res);
    return res;
  }

  return 0;
}

static int do_trik_fb_unset_format(struct fb_device* _fb) {
  if (_fb == NULL)
    return EINVAL;

  memset(&_fb->fix_info, 0, sizeof(_fb->fix_info));
  memset(&_fb->var_info, 0, sizeof(_fb->var_info));

  return 0;
}

static int do_trik_fb_get_info(struct fb_device* _fb, struct image_desc* _desc) {
  if (_fb == NULL || _desc == NULL)
    return EINVAL;

  _desc->width = _fb->var_info.xres;
  _desc->height = _fb->var_info.yres;
  _desc->line_len = _fb->fix_info.line_length;
  _desc->image_size = _fb->fix_info.smem_len;

#warning TODO check and get framebuffer format!
  _desc->pixel_format = V4L2_PIX_FMT_RGB565X;

  return 0;
}

static int do_trik_fb_mmap(struct fb_device* _fb) {
  int res;

  if (_fb == NULL)
    return EINVAL;

  _fb->size = _fb->fix_info.smem_len;
  _fb->buf = mmap(NULL, _fb->size, PROT_READ | PROT_WRITE, MAP_SHARED, _fb->fd, 0);
  if (_fb->buf == MAP_FAILED) {
    res = errno;
    fprintf(stderr, "mmap(%zu) failed: %d\n", _fb->size, res);
    return res;
  }

  return 0;
}

static int do_trik_fb_munmap(struct fb_device* _fb) {
  int res = 0;
  if (_fb->buf != MAP_FAILED) {
    if (munmap(_fb->buf, _fb->size) != 0) {
      res = errno;
      fprintf(stderr, "munmap(%p, %zu) failed: %d\n", _fb->buf, _fb->size, res);
    }
    _fb->buf = MAP_FAILED;
    _fb->size = 0;
  }

  return res;
}

static int do_trik_fb_get_frame(struct fb_device* _fb, void** _framePtr, size_t* _frameSize) {
  if (_fb == NULL || _framePtr == NULL || _frameSize == NULL)
    return EINVAL;

  if (_fb->buf == NULL)
    return ENOTCONN;

  *_framePtr = _fb->buf;
  *_frameSize = _fb->size;

  return 0;
}

int trik_fb_init(bool _verbose) {
  (void) _verbose;
  return 0;
}

int trik_fb_fini() { return 0; }

int trik_fb_open(struct fb_device* _fb, const char* path) {
  int res = 0;

  if (_fb == NULL)
    return EINVAL;
  if (_fb->fd != -1)
    return EALREADY;

  res = do_trik_fb_open(_fb, path);
  if (res != 0)
    goto exit;

  res = do_trik_fb_set_format(_fb);
  if (res != 0)
    goto exit_close;

  res = do_trik_fb_mmap(_fb);
  if (res != 0)
    goto exit_unset_format;

  return 0;

exit_unset_format:
  do_trik_fb_unset_format(_fb);
exit_close:
  do_trik_fb_close(_fb);
exit:
  return res;
}

int trik_fb_close(struct fb_device* _fb) {
  if (_fb == NULL)
    return EINVAL;
  if (_fb->fd == -1)
    return EALREADY;

  do_trik_fb_munmap(_fb);
  do_trik_fb_unset_format(_fb);
  do_trik_fb_close(_fb);

  return 0;
}

int trik_fb_start(struct fb_device* _fb) {
  if (_fb == NULL)
    return EINVAL;
  if (_fb->fd == -1)
    return ENOTCONN;

  return 0;
}

int trik_fb_stop(struct fb_device* _fb) {
  if (_fb == NULL)
    return EINVAL;
  if (_fb->fd == -1)
    return ENOTCONN;

  return 0;
}

int trik_fb_get_frame(struct fb_device* _fb, void** _framePtr, size_t* _frameSize) {
  if (_fb == NULL)
    return EINVAL;
  if (_fb->fd == -1)
    return ENOTCONN;

  return do_trik_fb_get_frame(_fb, _framePtr, _frameSize);
}

int trik_fb_put_frame(struct fb_device* _fb) {
  if (_fb == NULL)
    return EINVAL;
  if (_fb->fd == -1)
    return ENOTCONN;

  return 0;
}

int trik_fb_get_info(struct fb_device* _fb, struct image_desc* _desc) {
  if (_fb == NULL || _desc == NULL)
    return EINVAL;
  if (_fb->fd == -1)
    return ENOTCONN;

  return do_trik_fb_get_info(_fb, _desc);
}

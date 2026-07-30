#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <libv4l2.h>
#include <linux/videodev2.h>

#include "trik/sensors/module_v4l2.h"

static int do_trik_v4l2_open(struct v4l2_device* _v4l2, const char* _path) {
  int res;

  if (_v4l2 == NULL || _path == NULL)
    return EINVAL;

  _v4l2->fd = open(_path, O_RDWR | O_NONBLOCK, 0);
  if (_v4l2->fd < 0) {
    res = errno;
    fprintf(stderr, "trik_v4l2_open(%s) failed: %d\n", _path, res);
    _v4l2->fd = -1;
    return res;
  }

  return 0;
}

static int do_trik_v4l2_close(struct v4l2_device* _v4l2) {
  int res;
  if (_v4l2 == NULL)
    return EINVAL;

  if (close(_v4l2->fd) != 0) {
    res = errno;
    fprintf(stderr, "trik_v4l2_close() failed: %d\n", res);
    return res;
  }

  _v4l2->fd = -1;
  return 0;
}

static int do_trik_v4l2_set_format(struct v4l2_device* _v4l2, size_t _width, size_t _height, uint32_t _format) {
  int res;
  if (_v4l2 == NULL)
    return EINVAL;

  memset(&_v4l2->format, 0, sizeof(_v4l2->format));
  _v4l2->format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  _v4l2->format.fmt.pix.width = _width;
  _v4l2->format.fmt.pix.height = _height;
  _v4l2->format.fmt.pix.pixelformat = _format;
  _v4l2->format.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(_v4l2->fd, VIDIOC_TRY_FMT, &_v4l2->format) != 0)
    fprintf(stderr, "v4l2_ioctl(VIDIOC_TRY_FMT) failed: %d\n", errno);

  if (ioctl(_v4l2->fd, VIDIOC_S_FMT, &_v4l2->format) != 0) {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_S_FMT) failed: %d\n", res);
    return res;
  }
  
  // warn if format is emulated
  size_t fmtIdx;
  for (fmtIdx = 0;; ++fmtIdx) {
    struct v4l2_fmtdesc fmtDesc;
    fmtDesc.index = fmtIdx;
    fmtDesc.type = _v4l2->format.type;
    if (ioctl(_v4l2->fd, VIDIOC_ENUM_FMT, &fmtDesc) != 0) {
      // either fault or unknown format
      fprintf(stderr, "v4l2_ioctl(VIDIOC_ENUM_FMT) failed: %d\n", errno);
      break; // just warn, do not fail
    }

    if (fmtDesc.pixelformat == _v4l2->format.fmt.pix.pixelformat) {
      if (fmtDesc.flags & V4L2_FMT_FLAG_EMULATED)
        fprintf(stderr, "V4L2 format %c%c%c%c is emulated, performance will be degraded\n", (_v4l2->format.fmt.pix.pixelformat) & 0xff,
          (_v4l2->format.fmt.pix.pixelformat >> 8) & 0xff, (_v4l2->format.fmt.pix.pixelformat >> 16) & 0xff,
          (_v4l2->format.fmt.pix.pixelformat >> 24) & 0xff);
      break;
    }
  }

  return 0;
}

static int do_trik_v4l2_unset_format(struct v4l2_device* _v4l2) {
  if (_v4l2 == NULL)
    return EINVAL;

  memset(&_v4l2->format, 0, sizeof(_v4l2->format));

  return 0;
}

static int do_trik_v4l2_get_format(struct v4l2_device* _v4l2, struct image_desc* _desc) {
  if (_v4l2 == NULL || _desc == NULL)
    return EINVAL;

  _desc->width = _v4l2->format.fmt.pix.width;
  _desc->height = _v4l2->format.fmt.pix.height;
  _desc->line_len = _v4l2->format.fmt.pix.bytesperline;
  _desc->image_size = _v4l2->format.fmt.pix.sizeimage;
  _desc->pixel_format = _v4l2->format.fmt.pix.pixelformat;

  return 0;
}

static int do_trik_v4l2_mmap_bufs(struct v4l2_device* _v4l2) {
  int res = 0;

  if (_v4l2 == NULL)
    return EINVAL;

  struct v4l2_requestbuffers requestBuffers;
  memset(&requestBuffers, 0, sizeof(requestBuffers));
  requestBuffers.count = V4L2_BUF_COUNT;
  requestBuffers.type = _v4l2->format.type;
  requestBuffers.memory = V4L2_MEMORY_MMAP;

  if (ioctl(_v4l2->fd, VIDIOC_REQBUFS, &requestBuffers) != 0) {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) failed: %d\n", res);
    goto exit;
  }

  if (requestBuffers.count <= 0) {
    res = ENOSPC;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) returned no buffers\n");
    goto exit;
  } else if (requestBuffers.count < V4L2_BUF_COUNT)
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) returned only %" PRIu32 " buffers of %zu requested\n", requestBuffers.count,
      V4L2_BUF_COUNT);
  else if (requestBuffers.count > V4L2_BUF_COUNT) {
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) returned %" PRIu32 " buffers, used only %zu\n", requestBuffers.count,
      V4L2_BUF_COUNT);
    requestBuffers.count = V4L2_BUF_COUNT;
  }

  size_t bufferIndex;
  for (bufferIndex = 0; bufferIndex < requestBuffers.count; ++bufferIndex) {
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(buffer));
    buffer.index = bufferIndex;
    buffer.type = _v4l2->format.type;
    buffer.memory = V4L2_MEMORY_MMAP;

    if (ioctl(_v4l2->fd, VIDIOC_QUERYBUF, &buffer) != 0) {
      res = errno;
      fprintf(stderr, "v4l2_ioctl(VIDIOC_QUERYBUF, index %zu) failed: %d\n", bufferIndex, res);
      goto exit_unmap;
    }

    _v4l2->buf_size[bufferIndex] = buffer.length;
    _v4l2->buf[bufferIndex] = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, _v4l2->fd, buffer.m.offset);
    if (_v4l2->buf[bufferIndex] == MAP_FAILED) {
      res = errno;
      fprintf(stderr, "v4l2_mmap(index %zu, size %" PRIu32 ", offset %" PRIu32 ") failed: %d\n", bufferIndex, buffer.length, buffer.m.offset, res);
      goto exit_unmap;
    }
  }

  for (/*bufferIndex*/; bufferIndex < V4L2_BUF_COUNT; ++bufferIndex) { // fill unused buffers
    _v4l2->buf[bufferIndex] = MAP_FAILED;
    _v4l2->buf_size[bufferIndex] = 0;
  }

  return 0;

  size_t idx;
exit_unmap:
  for (idx = 0; idx < bufferIndex; ++idx)
    if (munmap(_v4l2->buf[idx], _v4l2->buf_size[idx]) != 0)
      // do not update res!
      fprintf(stderr, "v4l2_munmap(index %zu, ptr %p, size %zu) failed: %d\n", idx, _v4l2->buf[idx], _v4l2->buf_size[idx], errno);

exit:
  return res;
}

static int do_trik_v4l2_munmap_bufs(struct v4l2_device* _v4l2) {
  int res = 0;

  if (_v4l2 == NULL)
    return EINVAL;

  size_t bufferIndex;
  for (bufferIndex = 0; bufferIndex < V4L2_BUF_COUNT; ++bufferIndex) {
    if (_v4l2->buf[bufferIndex] != MAP_FAILED && munmap(_v4l2->buf[bufferIndex], _v4l2->buf_size[bufferIndex]) != 0) {
      res = errno; // last error will be returned
      fprintf(stderr, "v4l2_munmap(index %zu, ptr %p, size %zu) failed: %d\n", bufferIndex, _v4l2->buf[bufferIndex], _v4l2->buf_size[bufferIndex],
        res);
    }
    _v4l2->buf[bufferIndex] = MAP_FAILED;
    _v4l2->buf_size[bufferIndex] = 0;
  }

  return res;
}

static int do_trik_v4l2_start(struct v4l2_device* _v4l2) {
  int res = 0;

  if (_v4l2 == NULL)
    return EINVAL;

  size_t bufferIndex;
  for (bufferIndex = 0; bufferIndex < V4L2_BUF_COUNT; ++bufferIndex)
    if (_v4l2->buf[bufferIndex] != MAP_FAILED) {
      struct v4l2_buffer buffer;
      memset(&buffer, 0, sizeof(buffer));
      buffer.index = bufferIndex;
      buffer.type = _v4l2->format.type;
      buffer.memory = V4L2_MEMORY_MMAP;

      if (ioctl(_v4l2->fd, VIDIOC_QBUF, &buffer) != 0) {
        res = errno;
        fprintf(stderr, "v4l2_ioctl(VIDIOC_QBUF, index %zu) failed: %d\n", bufferIndex, res);
        goto exit_stop;
      }
    }

  _v4l2->frame_count = 0;

  enum v4l2_buf_type capture = _v4l2->format.type;
  if (ioctl(_v4l2->fd, VIDIOC_STREAMON, &capture) != 0) {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_STREAMON) failed: %d\n", res);
    goto exit_stop;
  }

  return 0;

exit_stop:
  capture = _v4l2->format.type;
  if (ioctl(_v4l2->fd, VIDIOC_STREAMOFF, &capture) != 0)
    // do not update res!
    fprintf(stderr, "v4l2_ioctl(VIDIOC_STREAMOFF) failed: %d\n", errno);

  // exit:
  return res;
}

static int do_trik_v4l2_stop(struct v4l2_device* _v4l2) {
  int res = 0;

  if (_v4l2 == NULL)
    return EINVAL;

  _v4l2->frame_count = 0;

  enum v4l2_buf_type capture = _v4l2->format.type;
  if (ioctl(_v4l2->fd, VIDIOC_STREAMOFF, &capture) != 0) {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_STREAMOFF) failed: %d\n", res);
    return res;
  }

  return 0;
}

static int do_trik_v4l2_get_frame(struct v4l2_device* _v4l2, const void** _framePtr, size_t* _frameSize, size_t* _frameIndex) {
  int res = 0;

  if (_v4l2 == NULL || _framePtr == NULL || _frameSize == NULL || _frameIndex == NULL)
    return EINVAL;

  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = _v4l2->format.type;
  buffer.memory = V4L2_MEMORY_MMAP;

  if (ioctl(_v4l2->fd, VIDIOC_DQBUF, &buffer) != 0) {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_DQBUF) failed: %d\n", res);
    return res;
  }

  if (buffer.index >= V4L2_BUF_COUNT || _v4l2->buf[buffer.index] == MAP_FAILED) {
    res = ECHRNG;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_DQBUF) returned invalid buffer index %" PRIu32 "\n", buffer.index);
    return res;
  }

  ++_v4l2->frame_count;

  *_frameIndex = buffer.index;
  *_framePtr = _v4l2->buf[buffer.index];
  *_frameSize = buffer.bytesused;

  return 0;
}

static int do_trik_v4l2_put_frame(struct v4l2_device* _v4l2, size_t _frameIndex) {
  int res = 0;

  if (_v4l2 == NULL)
    return EINVAL;

  if (_frameIndex >= V4L2_BUF_COUNT || _v4l2->buf[_frameIndex] == MAP_FAILED)
    return ECHRNG;

  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.index = _frameIndex;
  buffer.type = _v4l2->format.type;
  buffer.memory = V4L2_MEMORY_MMAP;

  if (ioctl(_v4l2->fd, VIDIOC_QBUF, &buffer) != 0) {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_QBUF, index %zu) failed: %d\n", _frameIndex, res);
    return res;
  }

  return 0;
}

int trik_v4l2_open(struct v4l2_device* _v4l2, const struct camera_config* _config) {
  int ret = 0;
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->fd != -1)
    return EALREADY;

  ret = do_trik_v4l2_open(_v4l2, _config->path);
  if (ret != 0)
    goto exit;

  ret = do_trik_v4l2_set_format(_v4l2, _config->width, _config->height, _config->format);
  if (ret != 0)
    goto exit_close;

  ret = do_trik_v4l2_mmap_bufs(_v4l2);
  if (ret != 0)
    goto exit_unset_format;

  return 0;

exit_unset_format:
  do_trik_v4l2_unset_format(_v4l2);
exit_close:
  do_trik_v4l2_close(_v4l2);
exit:
  return ret;
}

int trik_v4l2_close(struct v4l2_device* _v4l2) {
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->fd == -1)
    return EALREADY;

  do_trik_v4l2_munmap_bufs(_v4l2);
  do_trik_v4l2_unset_format(_v4l2);
  do_trik_v4l2_close(_v4l2);

  return 0;
}

int trik_v4l2_start(struct v4l2_device* _v4l2) {
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->fd == -1)
    return ENOTCONN;

  return do_trik_v4l2_start(_v4l2);
}

int trik_v4l2_stop(struct v4l2_device* _v4l2) {
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->fd == -1)
    return ENOTCONN;

  return do_trik_v4l2_stop(_v4l2);
}

int trik_v4l2_get_frame(struct v4l2_device* _v4l2, const void** _framePtr, size_t* _frameSize, size_t* _frameIndex) {
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->fd == -1)
    return ENOTCONN;

  return do_trik_v4l2_get_frame(_v4l2, _framePtr, _frameSize, _frameIndex);
}

int trik_v4l2_put_frame(struct v4l2_device* _v4l2, size_t _frameIndex) {
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->fd == -1)
    return ENOTCONN;

  return do_trik_v4l2_put_frame(_v4l2, _frameIndex);
}

int trik_v4l2_get_info(struct v4l2_device* _v4l2, struct image_desc* _desc) {
  if (_v4l2 == NULL || _desc == NULL)
    return EINVAL;
  if (_v4l2->fd == -1)
    return ENOTCONN;

  return do_trik_v4l2_get_format(_v4l2, _desc);
}

#ifndef TRIK_SENSORS_MODULE_V4L2_H_
#define TRIK_SENSORS_MODULE_V4L2_H_

#include <linux/videodev2.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "trik/sensors/config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Frame format descriptor — returned by the driver after VIDIOC_S_FMT.
 * width, height, line_len and image_size are filled by the kernel.
 */
struct image_desc {
  uint32_t width;
  uint32_t height;
  uint32_t line_len;       /* bytesperline */
  uint32_t image_size;     /* total frame size in bytes */
  uint32_t pixel_format;   /* V4L2 pixel format FourCC */
};

/*
 * Opened V4L2 device state. Holds the file descriptor, mmap'd DMA
 * ring buffers and the actual format negotiated with the driver.
 */
#define V4L2_BUF_COUNT 3

struct v4l2_device {
  int fd;
  unsigned long frame_count;
  struct v4l2_format format;
  void* buf[V4L2_BUF_COUNT];
  size_t buf_size[V4L2_BUF_COUNT];
};

/*
 * Open negotiating the given camera_config, then mmap DMA buffers.
 * Returns 0 on success.
 */
int  trik_v4l2_open(struct v4l2_device* dev, const struct camera_config* cfg);

/* Release mmap buffers and close the device. */
int  trik_v4l2_close(struct v4l2_device* dev);

/* Start streaming. Queues all buffers, then VIDIOC_STREAMON. */
int  trik_v4l2_start(struct v4l2_device* dev);

/* Stop streaming. VIDIOC_STREAMOFF. */
int  trik_v4l2_stop(struct v4l2_device* dev);

/*
 * Dequeue the next filled frame from the kernel ring.
 * idx — buffer index to return back with trik_v4l2_put_frame.
 */
int  trik_v4l2_get_frame(struct v4l2_device* dev, const void** ptr,
                    size_t* size, size_t* idx);

/* Return buffer back to the kernel ring after processing. */
int  trik_v4l2_put_frame(struct v4l2_device* dev, size_t idx);

/* Read back the actual frame format from the device. */
int  trik_v4l2_get_info(struct v4l2_device* dev, struct image_desc* desc);

#ifdef __cplusplus
}
#endif

#endif

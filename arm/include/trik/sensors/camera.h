#ifndef TRIK_SENSORS_CAMERA_H_
#define TRIK_SENSORS_CAMERA_H_

#include "trik/sensors/config.h"
#include "trik/sensors/module_v4l2.h"

/*
 * Runtime state of one physical camera.
 * Created by add_camera, destroyed by remove_camera.
 */
struct camera {
  int id;
  struct camera_config config;
  struct v4l2_device v4l2;
  bool ready;
  bool streaming;
};

/* Zero-initialize camera slot. */
void camera_init(struct camera* cam, int id);

/* Open camera with given config. Returns 0 on success, -1 on error. */
int camera_add(struct camera* cam, const struct camera_config* cfg);

/* Close V4L2 device, mark as not ready. */
void camera_remove(struct camera* cam);

#endif

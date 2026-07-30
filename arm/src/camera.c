#include <string.h>

#include "trik/sensors/camera.h"

void camera_init(struct camera* cam, int id)
{
  memset(cam, 0, sizeof(*cam));
  cam->id = id;
  cam->v4l2.fd = -1;
}

int camera_add(struct camera* cam, const struct camera_config* cfg)
{
  if (!cam || !cfg || !cfg->path)
    return -1;

  cam->config = *cfg;
  cam->ready = (trik_v4l2_open(&cam->v4l2, cfg) == 0);

  return cam->ready ? 0 : -1;
}

void camera_remove(struct camera* cam)
{
  if (cam && cam->ready) {
    trik_v4l2_close(&cam->v4l2);
    cam->ready = false;
  }
}

#ifndef TRIK_SENSORS_SCHEDULE_H_
#define TRIK_SENSORS_SCHEDULE_H_

#include <sys/select.h>

struct app;

struct schedule_slot {
  int cam_id;
  int ch_idx;
  int weight;
  int deficit;
};

/*
 * Weighted round-robin schedule across cameras and algorithms.
 * Each bound channel contributes weight slots to a flat playlist.
 * schedule_next() picks the next ready slot; skip if camera not ready.
 */
void schedule_rebuild(struct app* app);
struct schedule_slot* schedule_next(struct app* app, fd_set* fds);

#endif

#include <string.h>
#include <sys/select.h>

#include "trik/sensors/app.h"
#include "trik/sensors/schedule.h"

static struct schedule_slot g_slots[MAX_CHANNELS];
static int g_count;
static int g_cursor;

static void refill(void)
{
  for (int i = 0; i < g_count; i++)
    g_slots[i].deficit = g_slots[i].weight;
}

static bool all_zero(void)
{
  for (int i = 0; i < g_count; i++)
    if (g_slots[i].deficit > 0)
      return false;
  return true;
}

void schedule_rebuild(struct app* app)
{
  g_count = 0;
  for (int i = 0; i < MAX_CHANNELS; i++) {
    struct sensor_channel* ch = &app->channels[i];
    if (!ch->bound)
      continue;
    g_slots[g_count].cam_id  = ch->cam_id;
    g_slots[g_count].ch_idx  = i;
    g_slots[g_count].weight  = ch->priority;
    g_slots[g_count].deficit = ch->priority;
    g_count++;
  }
  g_cursor = 0;
}

struct schedule_slot* schedule_next(struct app* app, fd_set* fds)
{
  if (g_count == 0)
    return NULL;

  if (all_zero())
    refill();

  for (int tries = 0; tries < g_count; tries++) {
    struct schedule_slot* s = &g_slots[g_cursor];
    g_cursor = (g_cursor + 1) % g_count;

    if (s->deficit <= 0)
      continue;
    if (!app->cams[s->cam_id].ready)
      continue;
    if (!FD_ISSET(app->cams[s->cam_id].v4l2.fd, fds))
      continue;

    s->deficit--;
    return s;
  }

  return NULL;
}

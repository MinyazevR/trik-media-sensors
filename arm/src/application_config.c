#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "trik/sensors/config.h"

enum { OPT_CTRL_FIFO = 256, OPT_FB_PATH };

static const struct option longopts[] = {
  {"ctrl-fifo", 1, NULL, OPT_CTRL_FIFO},
  {"fb-path",   1, NULL, OPT_FB_PATH},
  {"help",      0, NULL, 'h'},
  {NULL, 0, NULL, 0}
};

void app_config_help(const char* arg0)
{
  fprintf(stderr,
    "Usage: %s --ctrl-fifo <path> [--fb-path <path>]\n"
    "\n"
    "  --ctrl-fifo <path>  Management FIFO for dynamic algorithm control.\n"
    "                       Accepts enable/disable/shutdown commands.\n"
    "  --fb-path <path>    Framebuffer device for video output.\n"
    "                       Default: %s\n",
    arg0, DEFAULT_FB_PATH);
}

int app_config_init(struct app_config* cfg, int argc, char* const argv[])
{
  int opt;

  if (!cfg)
    return -1;

  cfg->ctrl_fifo_path = NULL;
  cfg->fb_path = DEFAULT_FB_PATH;

  while ((opt = getopt_long(argc, argv, "h", longopts, NULL)) != -1) {
    switch (opt) {
    case OPT_CTRL_FIFO:
      cfg->ctrl_fifo_path = optarg;
      break;
    case OPT_FB_PATH:
      cfg->fb_path = optarg;
      break;
    case 'h':
      app_config_help(argv[0]);
      return -1;
    default:
      return -1;
    }
  }

  if (!cfg->ctrl_fifo_path) {
    fprintf(stderr, "Missing required argument: --ctrl-fifo\n");
    return -1;
  }

  return 0;
}

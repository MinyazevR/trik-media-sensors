#include <trik/sensors/dsp_server.h>

#define Registry_CURDESC Test__Desc
#define MODULE_NAME "Server"

#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>

#include <stdint.h>
#include <stdio.h>

#include <c6x.h>

#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <trik/buffer.h>
#include <trik/sensors/cmd.h>
#include <trik/sensors/cv_algorithm.h>
#include <trik/sensors/cv_algorithms.h>
#include <trik/sensors/msg.h>

int8_t __attribute__((aligned(128))) out_buff[BUFFER_SIZE];
int8_t __attribute__((aligned(128))) in_buff[BUFFER_SIZE];

typedef struct {
  UInt16 hostProcId;
  MessageQ_Handle slaveQue;
} Server_Module;

Registry_Desc Registry_CURDESC;
static Server_Module Module;

static enum trik_cv_algorithm cv_algorithm = TRIK_CV_ALGORITHM_NONE;

static struct buffer in_buffer;
static struct buffer out_buffer;

/*
 * DSP CPU clock (PLL0 SYSCLK1 = ARM CPU clock). Used only to convert the
 * free-running cycle counter (TSCL/TSCH) to microseconds. Must match the
 * board's actual frequency: TRIK ships 300 / 408 / 456 MHz depending on the
 * u-boot PLL setup. The raw cycle counts are also printed so the value can be
 * verified independently of this constant.
 */
#define DSP_CPU_FREQ_HZ 372000000u
#define TSC_TO_US(cycles) ((uint32_t)((uint64_t)(cycles) * 1000000u / DSP_CPU_FREQ_HZ))

/* Emit a per-algorithm report every N processed frames. */
#define STATS_REPORT_FRAMES 100u

#define ALGO_STATS_COUNT (TRIK_CV_ALGORITHM_JPEG_ENCODER + 1)

typedef struct {
  uint32_t frames;
  uint64_t cycles_total;
  uint32_t cycles_min;
  uint32_t cycles_max;
} AlgoStats;

static AlgoStats g_stats[ALGO_STATS_COUNT];
static uint32_t g_total_frames = 0;

static inline uint64_t tsc_read64(void) {
  uint32_t lo = (uint32_t) TSCL;
  uint32_t hi = (uint32_t) TSCH;
  uint32_t lo2 = (uint32_t) TSCL;
  if (lo2 < lo)
    hi = (uint32_t) TSCH;
  return ((uint64_t) hi << 32) | lo2;
}

static const char *algo_name(enum trik_cv_algorithm a) {
  switch (a) {
    case TRIK_CV_ALGORITHM_MOTION_SENSOR:    return "motion";
    case TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR: return "edge_line";
    case TRIK_CV_ALGORITHM_LINE_SENSOR:      return "line";
    case TRIK_CV_ALGORITHM_OBJECT_SENSOR:    return "object";
    case TRIK_CV_ALGORITHM_MXN_SENSOR:       return "mxn";
    case TRIK_CV_ALGORITHM_JPEG_ENCODER:     return "jpeg";
    default:                                 return "unknown";
  }
}

static void report_stats(void) {
  int i;
  /* Both Log_print and System_printf end up in the same SysMin trace buffer
   * (.tracebuf in DDR), read by the Linux remoteproc trace driver via
   * /sys/kernel/debug/remoteproc/remoteproc0/trace0. ti.trace.SysMin only
   * publishes a line once a '\n' is seen, so every format string ends with it.
   */
  System_printf("[DSP-stats] total frames=%u (assumed DSP clock %u Hz)\n",
                (unsigned) g_total_frames, (unsigned) DSP_CPU_FREQ_HZ);
  for (i = 0; i < ALGO_STATS_COUNT; ++i) {
    AlgoStats *s = &g_stats[i];
    uint32_t avg_us, min_us, max_us, max_fps;
    if (s->frames == 0)
      continue;
    avg_us = TSC_TO_US(s->cycles_total / s->frames);
    min_us = TSC_TO_US(s->cycles_min);
    max_us = TSC_TO_US(s->cycles_max);
    max_fps = avg_us > 0 ? 1000000u / avg_us : 0;
    System_printf("[DSP-stats] %s frames=%u avg=%u us min=%u us max=%u us -> %u fps\n",
                  algo_name((enum trik_cv_algorithm) i), (unsigned) s->frames,
                  (unsigned) avg_us, (unsigned) min_us, (unsigned) max_us,
                  (unsigned) max_fps);
  }
}

enum trik_cv_algorithm trik_cv_algorithm_from_cmd(enum trik_cmd cmd) {
  if (cmd == TRIK_CMD_MOTION_SENSOR)
    return TRIK_CV_ALGORITHM_MOTION_SENSOR;
  else if (cmd == TRIK_CMD_EDGE_LINE_SENSOR)
    return TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR;
  else if (cmd == TRIK_CMD_LINE_SENSOR)
    return TRIK_CV_ALGORITHM_LINE_SENSOR;
  else if (cmd == TRIK_CMD_OBJECT_SENSOR)
    return TRIK_CV_ALGORITHM_OBJECT_SENSOR;
  else if (cmd == TRIK_CMD_MXN_SENSOR)
    return TRIK_CV_ALGORITHM_MXN_SENSOR;
  else if (cmd == TRIK_CMD_JPEG_ENCODER)
    return TRIK_CV_ALGORITHM_JPEG_ENCODER;
  else
    return TRIK_CV_ALGORITHM_NONE;
}

Int trik_init_dsp_server(Void) {
  Int status = 0;
  MessageQ_Params msgqParams;
  char msgqName[32];
  Registry_Result result;

  Log_print0(Diags_ENTRY, "--> trik_init_dsp_server:");

  result = Registry_addModule(&Registry_CURDESC, MODULE_NAME);
  Assert_isTrue(result == Registry_SUCCESS, (Assert_Id) NULL);

  Module.hostProcId = MultiProc_getId("HOST");
  Diags_setMask(MODULE_NAME "+EXF");

  MessageQ_Params_init(&msgqParams);
  sprintf(msgqName, TRIK_SLAVE_MSG_QUE_NAME, MultiProc_getName(MultiProc_self()));
  Module.slaveQue = MessageQ_create(msgqName, &msgqParams);

  if (Module.slaveQue == NULL) {
    status = -1;
    goto leave;
  }

  Log_print0(Diags_INFO, "Server_create: server is ready");

leave:
  Log_print1(Diags_EXIT, "<-- trik_init_dsp_server: %d", (IArg) status);
  return (status);
}

int trik_destroy_dsp_server(Void) {
  int status;

  Log_print0(Diags_ENTRY, "--> trik_destroy_dsp_server:");

  status = MessageQ_delete(&Module.slaveQue);
  if (status < 0)
    goto leave;

leave:
  if (status < 0)
    Log_error1("Server_finish: error=0x%x", (IArg) status);

  Log_print1(Diags_EXIT, "<-- trik_destroy_dsp_server: %d", (IArg) status);
  Diags_setMask(MODULE_NAME "-EXF");

  return status;
}

static int trik_wait_for_msg(struct trik_msg** msg) {
  if (MessageQ_get(Module.slaveQue, (MessageQ_Msg*) msg, MessageQ_FOREVER) < 0)
    return -1;
  return 0;
}

static int trik_res_msg(struct trik_msg* msg) {
  MessageQ_QueueId queId = MessageQ_getReplyQueue(msg);
  MessageQ_put(queId, (MessageQ_Msg) msg);
  return 0;
}

static int trik_handle_init(struct trik_msg* req) {
  struct trik_res_init_msg* res = (struct trik_res_init_msg*) req;

  res->dsp_in_buffer = in_buffer.start;
  res->dsp_out_buffer = out_buffer.start;

  if (trik_res_msg((struct trik_msg*) res) < 0) {
    Log_print0(Diags_INFO, "trik_handle_init(): unable to send ack with buffers");
    return -1;
  }
  return 0;
}

static int trik_handle_sensor(struct trik_req_cv_algorithm_msg* req) {
  cv_algorithm = trik_cv_algorithm_from_cmd(req->header.cmd);

  struct trik_msg* res = (struct trik_msg*) req;

  if (!trik_init_cv_algorithm(cv_algorithm, req->video_format, req->line_length)) {
    Log_print1(Diags_INFO, "trik_handle_sensor(): unable to initialize cv algorithm %x", cv_algorithm);
    return -1;
  }
  Log_print1(Diags_INFO, "trik_handle_sensor(): Initialized %d algorithm", cv_algorithm);

  if (trik_res_msg(res) < 0) {
    Log_print0(Diags_INFO, "trik_handle_sensor(): unable to send ack about setting up motion sensor algo");
    return -1;
  }

  return 0;
}

static int trik_handle_step(struct trik_msg* req) {
  struct trik_res_step_msg* res = (struct trik_res_step_msg*) req;
  uint64_t t0, t1;
  uint32_t cycles;
  int ok;

  t0 = tsc_read64();
  ok = trik_run_cv_algorithm(cv_algorithm, in_buffer, out_buffer, res->in_args, &(res->out_args));
  t1 = tsc_read64();
  cycles = (uint32_t) (t1 - t0);
  res->out_args.process_time_us = TSC_TO_US(cycles);

  if (ok && cv_algorithm >= 0 && cv_algorithm < ALGO_STATS_COUNT) {
    AlgoStats* s = &g_stats[cv_algorithm];
    s->frames++;
    s->cycles_total += cycles;
    if (s->cycles_min == 0 || cycles < s->cycles_min)
      s->cycles_min = cycles;
    if (cycles > s->cycles_max)
      s->cycles_max = cycles;

    g_total_frames++;
    if ((g_total_frames % STATS_REPORT_FRAMES) == 0)
      report_stats();
  }

  if (!ok) {
    Log_print0(Diags_INFO, "trik_handle_step(): unable to run cv algorithm");
    return -1;
  }

  if (trik_res_msg((struct trik_msg*) res) < 0) {
    Log_print0(Diags_INFO, "trik_handle_step(): unable to send ack about step");
    return -1;
  }
  return 0;
}

Int trik_start_dsp_server(Void) {
  Int status = 0;
  Bool running = TRUE;
  struct trik_msg* msg;

  Log_print0(Diags_ENTRY | Diags_INFO, "--> trik_start_dsp_server");

  in_buffer.start = (void*) &in_buff;
  in_buffer.length = BUFFER_SIZE;
  out_buffer.start = (void*) &out_buff;
  out_buffer.length = BUFFER_SIZE;

  while (running) {
    status = trik_wait_for_msg(&msg);
    if (status < 0)
      goto leave;
    if (msg->cmd == TRIK_CMD_INIT) {
      if (trik_handle_init(msg) < 0) {
        printf("trik_start_dsp_server(): unable to handle init command");
        return -1;
      }
    } else if (msg->cmd == TRIK_CMD_STEP) {
      if (trik_handle_step(msg) < 0) {
        printf("trik_start_dsp_server(): unable to handle step command");
        return -1;
      }
    } else if (msg->cmd == TRIK_CMD_SHUTDOWN) {
      running = FALSE;
    } else if (msg->cmd != TRIK_CMD_NOP) {
      if (trik_handle_sensor((struct trik_req_cv_algorithm_msg*) msg) < 0) {
        printf("trik_start_dsp_server(): unable to handle motion sensor command");
        return -1;
      }
    }
  }

leave:
  Log_print1(Diags_EXIT, "<-- trik_start_dsp_server: %d", (IArg) status);
  return (status);
}

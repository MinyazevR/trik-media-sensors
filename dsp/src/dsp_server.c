#include <trik/sensors/dsp_server.h>

#define Registry_CURDESC Test__Desc
#define MODULE_NAME "Server"

#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>
#include <xdc/std.h>

#include <stdio.h>

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

  if (!trik_run_cv_algorithm(cv_algorithm, in_buffer, out_buffer, res->in_args, &(res->out_args))) {
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

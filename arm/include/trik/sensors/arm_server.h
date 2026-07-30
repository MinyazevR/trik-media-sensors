#ifndef TRIK_SENSORS_ARM_SERVER_
#define TRIK_SENSORS_ARM_SERVER_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <trik/buffer.h>
#include <trik/sensors/cv_algorithm.h>

int trik_init_arm_server(uint16_t rproc_id);
int trik_destroy_arm_server(void);

int trik_req_step(struct trik_cv_algorithm_out_args* out_args, struct trik_cv_algorithm_in_args in_args);

int trik_dsp_init_buffer(struct buffer* in, struct buffer* out);
int trik_dsp_register_algo(enum trik_cv_algorithm algo, uint32_t v4l2_fmt, uint32_t line_len);
#ifdef __cplusplus
}
#endif

#endif

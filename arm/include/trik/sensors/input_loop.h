#ifndef TRIK_SENSORS_INPUT_LOOP_H_
#define TRIK_SENSORS_INPUT_LOOP_H_

struct app;

/*
 * Main event loop — reads the management FIFO, dispatches commands.
 * Runs in a dedicated thread, blocks until shutdown is received.
 */
void input_loop_run(struct app* app);

#endif

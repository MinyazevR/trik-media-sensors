#ifndef TRIK_SENSORS_VIDEO_LOOP_H_
#define TRIK_SENSORS_VIDEO_LOOP_H_

struct app;

/*
 * Video capture and processing loop.
 * Owns display and DSP buffers, runs the weighted schedule across
 * cameras and sensors. Blocks until app->terminate.
 */
void video_loop_run(struct app* app);

#endif

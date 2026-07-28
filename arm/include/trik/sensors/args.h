#ifndef TRIK_SENSORS_ARGS_H_
#define TRIK_SENSORS_ARGS_H_

#include "trik/sensors/runtime.h"

bool argsParse(Runtime* _runtime, int _argc, char* const _argv[]);
void argsHelp(Runtime* _runtime, const char* _arg0);

#endif


#ifndef _TASKS_H_
#define _TASKS_H_

#include <scmRTOS.h>

// Process types
typedef OS::process<OS::pr0, 300> ThandleTSC;
typedef OS::process<OS::pr1, 300> ThandleGPIO;


#endif // _TASKS_H_

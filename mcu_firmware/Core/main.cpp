
#include "scmRTOS.h"
#include "board.h"


int main()
{
    initMCU();

    // run tasks
    OS::run();
}

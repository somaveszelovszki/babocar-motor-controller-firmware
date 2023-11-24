#include <micro/debug/DebugLed.hpp>
#include <micro/debug/TaskMonitor.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <globals.hpp>

using namespace micro;

extern "C" void runDebugTask(void) {
    taskMonitor.registerInitializedTask();

    DebugLed debugLed(gpio_Led);

    while (true) {
        debugLed.update(taskMonitor.ok());
        taskMonitor.notify(true);
        os_sleep(millisecond_t(1));
    }
}

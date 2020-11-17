#include <cfg_board.hpp>
#include <micro/debug/DebugLed.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/utils/timer.hpp>

using namespace micro;

extern "C" void runDebugTask(void) {

    SystemManager::instance().registerTask();

    DebugLed debugLed(gpio_Led);

    while (true) {
        debugLed.update(SystemManager::instance().failingTasks().size() == 0);
        SystemManager::instance().notify(true);
        os_sleep(millisecond_t(1));
    }
}

#include <cfg_board.h>
#include <globals.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

namespace {

} // namespace

extern "C" void runSensorTask(void) {

    while (true) {

    }

    vTaskDelete(nullptr);
}

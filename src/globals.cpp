#include <cfg_board.hpp>
#include <cfg_system.hpp>
#include <globals.hpp>

using namespace micro;

queue_t<RemoteControllerData, 1> remoteControllerQueue;

CanManager vehicleCanManager(can_Vehicle);
TaskMonitor taskMonitor(cfg::NUM_MONITORED_TASKS);

bool useSafetyEnableSignal = true;

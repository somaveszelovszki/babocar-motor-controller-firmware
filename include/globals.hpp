#pragma once

#include "RemoteControllerData.hpp"

#include <micro/debug/TaskMonitor.hpp>
#include <micro/panel/CanManager.hpp>

extern micro::queue_t<RemoteControllerData, 1> remoteControllerQueue;

extern micro::CanManager vehicleCanManager;
extern micro::TaskMonitor taskMonitor;

extern bool useSafetyEnableSignal;

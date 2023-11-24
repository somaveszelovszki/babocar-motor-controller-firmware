#pragma once

#include <micro/debug/TaskMonitor.hpp>
#include <micro/panel/CanManager.hpp>

#include "RemoteControllerData.hpp"

extern micro::queue_t<RemoteControllerData, 1> remoteControllerQueue;

extern micro::CanManager vehicleCanManager;
extern micro::TaskMonitor taskMonitor;

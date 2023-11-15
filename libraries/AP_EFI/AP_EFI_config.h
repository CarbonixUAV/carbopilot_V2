#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Scripting/AP_Scripting.h>

#ifndef AP_EFI_BACKEND_DEFAULT_ENABLED
#define AP_EFI_BACKEND_DEFAULT_ENABLED 1
#endif

#ifndef HAL_EFI_ENABLED
#define HAL_EFI_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif

#ifndef AP_EFI_SCRIPTING_ENABLED
#define AP_EFI_SCRIPTING_ENABLED (HAL_EFI_ENABLED && AP_SCRIPTING_ENABLED)
#endif

#ifndef AP_EFI_SERIAL_HIRTH_ENABLED
#define AP_EFI_SERIAL_HIRTH_ENABLED AP_EFI_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_EFI_THROTTLE_LINEARISATION_ENABLED
#define AP_EFI_THROTTLE_LINEARISATION_ENABLED AP_EFI_SERIAL_HIRTH_ENABLED
#endif

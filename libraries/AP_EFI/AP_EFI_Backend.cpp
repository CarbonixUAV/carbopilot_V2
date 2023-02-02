/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_EFI.h"

#if HAL_EFI_ENABLED

#include "AP_EFI_Backend.h"

extern const AP_HAL::HAL &hal;

AP_EFI_Backend::AP_EFI_Backend(AP_EFI &_frontend) :
    frontend(_frontend)
{
}

void AP_EFI_Backend::copy_to_frontend() 
{
    WITH_SEMAPHORE(sem);
    frontend.state = internal_state;
}

float AP_EFI_Backend::get_coef1(void) const
{
    return frontend.coef1;
}

float AP_EFI_Backend::get_coef2(void) const
{
    return frontend.coef2;
}

float AP_EFI_Backend::get_throttle_scale(void) const
{
    return frontend.throttle_scale;
}

float AP_EFI_Backend::get_throttle_idle(void) const
{
    return frontend.throttle_idle;
}

float AP_EFI_Backend::get_throttle_max(void) const
{
    return frontend.throttle_max;
}

float AP_EFI_Backend::get_ecu_fcr_slope(void) const
{
    return frontend.ecu_fcr_slope;
}

float AP_EFI_Backend::get_ecu_fcr_offset(void) const
{
    return frontend.ecu_fcr_offset;
}

int16_t AP_EFI_Backend::get_ecu_fcr_average_count(void) const
{
    return frontend.ecu_fcr_average_count;
}

int16_t AP_EFI_Backend::get_fuel_volume_in_ml(void) const
{
    return frontend.fuel_volume_in_ml;
}

#endif // HAL_EFI_ENABLED

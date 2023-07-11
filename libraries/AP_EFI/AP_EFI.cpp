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

#include "AP_EFI_Serial_MS.h"
#include "AP_EFI_Serial_Lutan.h"
#include "AP_EFI_NWPMU.h"
#include "AP_EFI_Serial_Hirth.h"
#include <AP_Logger/AP_Logger.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_CANManager/AP_CANManager.h>
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_EFI::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: EFI communication type
    // @Description: What method of communication is used for EFI #1
    // @Values: 0:None,1:Serial-MS,2:NWPMU,3:Serial-Lutan
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_EFI, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _COEF1
    // @DisplayName: EFI Calibration Coefficient 1
    // @Description: Used to calibrate fuel flow for MS protocol (Slope)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("_COEF1", 2, AP_EFI, coef1, 0),

    // @Param: _COEF2
    // @DisplayName: EFI Calibration Coefficient 2
    // @Description: Used to calibrate fuel flow for MS protocol (Offset)
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("_COEF2", 3, AP_EFI, coef2, 0),

    // @Param: _THTL_FO
    // @DisplayName: Throttle value - First Order 
    // @Description:  First Order Polynomial. (=1, if throttle is first order polynomial trendline)
    // @Values: floating values with 0.0001 Resolution. Can be -ve
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_THTL_FOP", 4, AP_EFI, throttle_firstorder, 1),

    // @Param: _THTL_SOP
    // @DisplayName: Throttle value - Second Order 
    // @Description:  Second Order Polynomial. (=0, if throttle is first order polynomial trendline)
    // @Values: floating values with 0.0001 Resolution. Can be -ve
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_THTL_SOP", 5, AP_EFI, throttle_secondorder, 0),

    // @Param: _THTL_TOP
    // @DisplayName: Throttle value - First Order 
    // @Description:  Third Order Polynomial. (=0, if throttle is first order polynomial trendline)
    // @Values: floating values with 0.0001 Resolution. Can be -ve
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_THTL_TOP", 6, AP_EFI, throttle_thirdorder, 0),

    // @Param: _THTL_OFF
    // @DisplayName: EFI throttle linearization offset
    // @Description: Offset for throttle linearization 
    // @Values: 0 - 100 (0.1 Resolution)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_THTL_OFF", 7, AP_EFI, throttle_offset, 10),

    // @Param: _EFCR_SLP
    // @DisplayName: ECU Fuel Consumption Rate factor
    // @Description: ECU FCR gradient/factor. Must be used along with _EFCR_OFT
    // @Values: 0 - 1000 (0.1 Resolution)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_EFCR_SLP", 8, AP_EFI, ecu_fcr_slope, 1),

    // @Param: _EFCR_OFT
    // @DisplayName: ECU Fuel Consumption Rate Offset
    // @Description: ECU FCR intercept/offset. Must be used along with _EFCR_SLP
    // @Values: 0 - 1000 (0.1 Resolution)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_EFCR_OFT", 9, AP_EFI, ecu_fcr_offset, 0),

    // @Param: _EFCR_AVG
    // @DisplayName: ECU Fuel Consumption Rate Average count
    // @Description: Averages _EFCR_AVG consecutive reading
    // @Values: 0 - 100 (1 Resolution)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_EFCR_AVG", 10, AP_EFI, ecu_fcr_average_count, 1),

    // @Param: _FUEL_VOL
    // @DisplayName: Full Fuel Volume / Capacity
    // @Description: Full fuel volume in ml
    // @Values: 0 - 65535 (1 Resolution)
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_FUEL_VOL", 11, AP_EFI, fuel_volume_in_ml, 1),

    AP_GROUPEND
};

AP_EFI *AP_EFI::singleton;

// Initialize parameters
AP_EFI::AP_EFI()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Initialize backends based on existing params
void AP_EFI::init(void)
{
    if (backend != nullptr) {
        // Init called twice, perhaps
        return;
    }
    switch ((Type)type.get()) {
    case Type::NONE:
        break;
    case Type::MegaSquirt:
        backend = new AP_EFI_Serial_MS(*this);
        break;
    case Type::Lutan:
        backend = new AP_EFI_Serial_Lutan(*this);
        break;
    case Type::NWPMU:
#if HAL_EFI_NWPWU_ENABLED
        backend = new AP_EFI_NWPMU(*this);
#endif
        break;
    case Type::Hirth:
        backend = new AP_EFI_Serial_Hirth(*this);
        break;
    default:
        gcs().send_text(MAV_SEVERITY_INFO, "Unknown EFI type");
        break;
    }
}

// Ask all backends to update the frontend
void AP_EFI::update()
{
    if (backend) {
        backend->update();
        log_status();
    }
}

bool AP_EFI::is_healthy(void) const
{
    return (backend && (AP_HAL::millis() - state.last_updated_ms) < HEALTHY_LAST_RECEIVED_MS);
}

/*
  write status to log
 */
void AP_EFI::log_status(void)
{
    AP::logger().WriteStreaming("EFI",
                       "TimeUS,LP,Rpm,SDT,ATM,IMP,IMT,ECT,OilP,OilT,FP,FCR,CFV,TPS,IDX",
                       "s%qsPPOOPOP--%-",
                       "F00C--00-0-0000",
                       "QBIffffffffffBB",
                       AP_HAL::micros64(),
                       uint8_t(state.engine_load_percent),
                       uint32_t(state.engine_speed_rpm),
                       float(state.spark_dwell_time_ms),
                       float(state.atmospheric_pressure_kpa),
                       float(state.intake_manifold_pressure_kpa),
                       float(state.intake_manifold_temperature),
                       float(state.coolant_temperature),
                       float(state.oil_pressure),
                       float(state.oil_temperature),
                       float(state.fuel_pressure),
                       float(state.fuel_consumption_rate_cm3pm),
                       float(state.estimated_consumed_fuel_volume_cm3),
                       uint8_t(state.throttle_position_percent),
                       uint8_t(state.ecu_index));

    AP::logger().WriteStreaming("EFI2",
                    "TimeUS,H,ES,SF,ETS,ATS,APS,TS,LCt,CT1_E,CT2_E,FR,FT,FA,IDX",
                    "s--------------",
                    "F--------------",
                    "QBBBBBBBBBBfffB",
                    AP_HAL::micros64(),
                    uint8_t(is_healthy()),
                    uint8_t(state.engine_state),
                    uint8_t(state.save_in_flash),
                    uint8_t(state.engine_temperature_sensor_status),
                    uint8_t(state.air_temperature_sensor_status),
                    uint8_t(state.air_pressure_sensor_status),
                    uint8_t(state.throttle_sensor_status),
                    uint8_t(state.no_of_log_data),
                    uint8_t(state.CHT_1_error_excess_temperature_status),
                    uint8_t(state.CHT_2_error_excess_temperature_status),
                    float(state.fuel_consumption_rate_raw),
                    float(state.total_fuel_consumed),
                    float(state.fuel_consumption_rate_average),
                    uint8_t(state.ecu_index));

    AP::logger().WriteStreaming("EFI3",
                    "TimeUS,E1_E,E2_E,C1_T,C2_T,E1_T,E2_T,k_th,thr_f,a_t,e_t,IDX",
                    "s--OOOOOOOO-",
                    "F--00000000-",
                    "QBBffffffffB",
                    AP_HAL::micros64(),
                    uint8_t(state.EGT_1_error_excess_temperature_status),
                    uint8_t(state.EGT_2_error_excess_temperature_status),
                    float(state.cht1_temp),
                    float(state.cht2_temp),
                    float(state.egt1_temp),
                    float(state.egt2_temp),
                    float(state.k_throttle),
                    float(state.thr_pos),
                    float(state.air_temp),
                    float(state.eng_temp),
                    uint8_t(state.ecu_index));

    AP::logger().WriteStreaming("EFI4",
                    "TimeUS,BVOL,crc,uptime,lpc,ack,pkt,at,a1,a2,a3,IDX",
                    "sv----------",
                    "F-----------",
                    "QfIIIIIIIIIB",
                    AP_HAL::micros64(),
                    float(state.battery_voltage),
                    uint32_t(state.crc_fail_cnt),
                    uint32_t(state.uptime),
                    uint32_t(state.loop_cnt),
                    uint32_t(state.ack_fail_cnt),
                    uint32_t(state.packet_sent),
                    uint32_t(state.ack_thr),
                    uint32_t(state.ack_s1),
                    uint32_t(state.ack_s2),
                    uint32_t(state.ack_s3),
                    uint8_t(state.ecu_index));


    for (uint8_t i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
// @LoggerMessage: ECYL
// @Description: EFI per-cylinder information
// @Field: TimeUS: Time since system startup
// @Field: Inst: Cylinder this data belongs to
// @Field: IgnT: Ignition timing
// @Field: InjT: Injection time
// @Field: CHT: Cylinder head temperature
// @Field: EGT: Exhaust gas temperature
// @Field: Lambda: Estimated lambda coefficient (dimensionless ratio)
// @Field: IDX: Index of the publishing ECU
        AP::logger().WriteStreaming("ECYL",
                           "TimeUS,Inst,IgnT,InjT,CHT,EGT,Lambda,IDX",
                           "s#dsOO--",
                           "F-0C0000",
                           "QBfffffB",
                           AP_HAL::micros64(),
                           i,
                           state.cylinder_status[i].ignition_timing_deg,
                           state.cylinder_status[i].injection_time_ms,
                           state.cylinder_status[i].cylinder_head_temperature,
                           state.cylinder_status[i].exhaust_gas_temperature,
                           state.cylinder_status[i].lambda_coefficient,
                           state.ecu_index);
    }
}

/*
  send EFI_STATUS
 */
void AP_EFI::send_mavlink_status(mavlink_channel_t chan)
{
    if (!backend) {
        return;
    }
    // Adding in new requested ECU telemetry fields for Hirth on 18/1/23 for live monitoring
    // EGT and CGT variables are already in Celcius from Hirth
    mavlink_msg_efi_status_send(
        chan,
        AP_EFI::is_healthy(),
        state.ecu_index,
        state.engine_speed_rpm,
        state.estimated_consumed_fuel_volume_cm3,
        state.fuel_consumption_rate_cm3pm,
        state.egt2_temp,        //was state.engine_load_percent, //EGT2
        state.throttle_position_percent, //throttle position
        state.spark_dwell_time_ms, //TBD
        state.cht2_temp, //was barometric pressure/state.atmospheric_pressure_kpa, //CHT2
        state.converted_map, //was state.intake_manifold_pressure_kpa
        state.air_temp, //was KELVIN_TO_C(state.intake_manifold_temperature),
        state.cht1_temp, //KELVIN_TO_C(state.cylinder_status[0].cylinder_head_temperature), //CHT1
        state.cylinder_status[0].ignition_timing_deg,
        state.cylinder_status[0].injection_time_ms,
        state.egt1_temp, //EGT1
        state.thr_pos, //throttle_out from 0 - 100
        float(state.engine_state)); //pt_compensation
}

namespace AP {
AP_EFI *EFI()
{
    return AP_EFI::get_singleton();
}
}

#endif // HAL_EFI_ENABLED

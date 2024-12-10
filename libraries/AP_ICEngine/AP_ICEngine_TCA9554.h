/*
  optional control of starter via a TCA9554 I2C
 */

#include "AP_ICEngine_config.h"

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
#include "AP_ICEngine.h"

class AP_ICEngine_TCA9554 {
public:
    void set_starter(bool on, AP_Int8 crank_direction);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_TCA9554;

    uint8_t last_state;

    bool initialised;

    bool TCA9554_init();
    void TCA9554_set(uint8_t value);
    uint32_t last_reg_check_ms;
};

#endif // AP_ICENGINE_TCA9554_STARTER_ENABLED

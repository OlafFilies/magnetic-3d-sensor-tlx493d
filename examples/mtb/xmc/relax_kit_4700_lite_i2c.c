// std includes

// MTB includes
#include "cybsp.h"
#include "cy_utils.h"

// XMC includes
#include "xmc_i2c.h"


#define SDA_PIN P3_15
#define SCL_PIN P0_13


XMC_GPIO_CONFIG_t i2c_sda =
{
  .mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT2,
  .output_strength = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
};

XMC_GPIO_CONFIG_t i2c_scl =
{
  .mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT2,
  .output_strength = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM
};


XMC_I2C_CH_CONFIG_t i2c_cfg =
{
  .baudrate = 100000U,
  .address = 0U
};


void setup_i2c_relax_kit_4700_lite(void) {
    XMC_I2C_CH_Init(XMC_I2C1_CH1, &i2c_cfg);
    XMC_I2C_CH_SetInputSource(XMC_I2C1_CH1, XMC_I2C_CH_INPUT_SDA, USIC1_C1_DX0_P3_15);
    XMC_I2C_CH_SetInputSource(XMC_I2C1_CH1, XMC_I2C_CH_INPUT_SCL, USIC1_C1_DX1_P0_13);
    XMC_I2C_CH_Start(XMC_I2C1_CH1);

    XMC_GPIO_Init(SCL_PIN, &i2c_scl);
    XMC_GPIO_Init(SDA_PIN, &i2c_sda);
}

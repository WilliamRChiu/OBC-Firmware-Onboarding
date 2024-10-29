#include "lm75bd.h"
#include "i2c_io.h"
#include "errors.h"
#include "logging.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

/* LM75BD Registers (p.8) */
#define LM75BD_REG_CONF 0x01U  /* Configuration Register (R/W) */

error_code_t lm75bdInit(lm75bd_config_t *config) {
  error_code_t errCode;

  if (config == NULL) return ERR_CODE_INVALID_ARG;

  RETURN_IF_ERROR_CODE(writeConfigLM75BD(config->devAddr, config->osFaultQueueSize, config->osPolarity,
                                         config->osOperationMode, config->devOperationMode));

  // Assume that the overtemperature and hysteresis thresholds are already set
  // Hysteresis: 75 degrees Celsius
  // Overtemperature: 80 degrees Celsius

  return ERR_CODE_SUCCESS;
}

error_code_t readTempLM75BD(uint8_t devAddr, float *temp) {
  error_code_t errCode;
  if (temp == NULL) {
    return ERR_CODE_INVALID_ARG;
  }

  /*Set the Pointer Register to the Temperature Register */
  uint8_t tempRegAddr = 0x00;
  errCode = i2cSendTo(devAddr, &tempRegAddr, sizeof(tempRegAddr));
  if (errCode != ERR_CODE_SUCCESS) {
    LOG_ERROR("Could not send Temperature. Error Code: %d", errCode);
    return errCode;
  }

  /*Read the Temperature Data from the Sensor */
  uint8_t tempBuffer[2] = {0};
  errCode = i2cReceiveFrom(devAddr, tempBuffer, sizeof(tempBuffer));
  if (errCode != ERR_CODE_SUCCESS) {
    LOG_ERROR("Could not read Temperature. Error Code: %d", errCode);
    return errCode;
  }

  /*Convert the Raw Data to Degrees Celsius */
  // Combine two bytes into a single 16-bit signed integer
  //shift the bit over by 8 to multiply it by 2 8 times
  //need to combine the most significant byte with the least significant byte
  int16_t rawTemp = (int16_t)((tempBuffer[0] << 8) | tempBuffer[1]);
  
  // Perform the conversion using the conversion factor
  //as temperature can be both positive and negative, represent it using two's complement
  //since temperature is represented in the upper 11 bits
  //therefore, need to shift the rawTemp over to the right by 5 bits
  //each LSB is 0.125 degrees
  *temp = (rawTemp >> 5) * 0.125;

  LOG_INFO("Temperature read from LM75BD: %.3f°C", *temp);
 
  return ERR_CODE_SUCCESS;
}

#define CONF_WRITE_BUFF_SIZE 2U
error_code_t writeConfigLM75BD(uint8_t devAddr, uint8_t osFaultQueueSize, uint8_t osPolarity,
                                   uint8_t osOperationMode, uint8_t devOperationMode) {
  error_code_t errCode;

  // Stores the register address and data to be written
  // 0: Register address
  // 1: Data
  uint8_t buff[CONF_WRITE_BUFF_SIZE] = {0};

  buff[0] = LM75BD_REG_CONF;

  uint8_t osFaltQueueRegData = 0;
  switch (osFaultQueueSize) {
    case 1:
      osFaltQueueRegData = 0;
      break;
    case 2:
      osFaltQueueRegData = 1;
      break;
    case 4:
      osFaltQueueRegData = 2;
      break;
    case 6:
      osFaltQueueRegData = 3;
      break;
    default:
      return ERR_CODE_INVALID_ARG;
  }

  buff[1] |= (osFaltQueueRegData << 3);
  buff[1] |= (osPolarity << 2);
  buff[1] |= (osOperationMode << 1);
  buff[1] |= devOperationMode;

  errCode = i2cSendTo(LM75BD_OBC_I2C_ADDR, buff, CONF_WRITE_BUFF_SIZE);
  if (errCode != ERR_CODE_SUCCESS) return errCode;

  return ERR_CODE_SUCCESS;
}

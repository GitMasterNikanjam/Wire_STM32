#pragma once

// Define the target MCU family here
#define STM32F4
// #define STM32F1
// #define STM32H7

// ###############################################################################################
// Include libraries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#endif

#include <string>
#include <inttypes.h>

// ###############################################################################################

#define BUFFER_LENGTH 32  // Buffer size for data transfer

// ##############################################################################################

class Wire {
public:

    /**
     * @brief Default constructor.
     */
    Wire(I2C_HandleTypeDef *i2cHandle);

    /**
     * @brief Initialize I2C
     */
    void begin();           

    /**
     * @brief Start communication with a device
     *  */                 
    void beginTransmission(uint8_t address); 

    /**
     * @brief End communication
     */
    uint8_t endTransmission();               

    /**
     * @brief Request data from a device
     */
    uint8_t requestFrom(uint8_t address, uint8_t quantity); 

    /**
     * @brief Write a byte
     */
    size_t write(uint8_t data);              

    /**
     * @brief Write multiple bytes
     */
    size_t write(const uint8_t* data, size_t length); 

    /**
     * @brief Check how many bytes are available to read
     */
    int available();               

    /**
     * @brief Read a byte
     *  */          
    int read();                              

private:

    /**
     * @brief HAL I2C handle
     */
    I2C_HandleTypeDef *_hi2c1;        

    /// @brief Receive buffer   
    uint8_t _rxBuffer[BUFFER_LENGTH];   

    /// @brief Current index in the receive buffer     
    uint8_t _rxIndex;       

    /// @brief Length of received data                 
    uint8_t _rxLength;         

    /// @brief Address of the target device              
    uint8_t _txAddress;                      
};

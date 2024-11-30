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
// #include <inttypes.h>

// ###############################################################################################
// Define Global Macros:

#define WIRE_BUFFER_LENGTH 32  // Buffer size for data transfer

// #define UNDER_DEVELOP

// ##############################################################################################

/**
 * @class Wire
 */
class Wire 
{
public:

    /// @brief Last error occurred for the object.
    std::string errorMessage;

    /**
     * @brief Constructor.
     * @param i2cHandle is a HAL I2C_HandleTypeDef pointer.
     * @note - The i2cHandle can be defined outside of the Wire class and then used within the class.
     * 
     * @note - It is recommended that the I2C peripheral be set and initialized outside of the Wire class.
     * 
     * @note - The GPIO I2C peripheral configurations must be set outside of the Wire class.
     */
    Wire(I2C_HandleTypeDef *i2cHandle);

    /**
     * @brief Constructor.
     * @note - The default instance of i2cHandle is I2C1.
     * 
     * @note - It is recommended that the I2C1 peripheral be set and initialized outside of the Wire class.
     * 
     * @note - The GPIO I2C peripheral configurations must be set outside of the Wire class.
     */
    Wire();

    /**
     * @brief Initialize I2C
     * @return true if succeeded.
     */
    bool begin();           

    /**
     * @brief Set Clock speed of i2c prepheral.
     * @return true if succeeded.
     */
    bool setClock(uint32_t clock);

    /**
     * @brief Start communication with a device
     *  */                 
    void beginTransmission(uint8_t address); 

    /**
     * @brief End communication and send data to I2C device.
     */
    bool endTransmission();               

    /**
     * @brief Request data from an I2C device.
     * @return true if succeeded.
     */
    bool requestFrom(uint8_t address, uint8_t quantity); 

    /**
     * @brief Write a byte
     * @return true if byte is written successfully.
     */
    bool write(uint8_t data);              

    /**
     * @brief Write multiple bytes
     * @return true if all bytes are written successfully.
     */
    bool write(const uint8_t* data, size_t length); 

    /**
     * @brief Check how many bytes are available to read
     * @return int Number of bytes available.
     */
    int available();               

    /**
     * @brief Read a byte
     * @return int The byte value or -1 if no data is available.
     *  */          
    int read();       

    #if UNDER_DEVELOP

    /**
     * @brief Initialize I2C in Slave Mode with a specified address.
     * @param address The I2C address for the slave device.
     * @return true if succeeded.
     */
    bool beginSlave(uint8_t address); 

    /**
     * @brief Start receiving data as a slave.
     * @param quantity Number of bytes to receive.
     * @return true if succeeded.
     */
    bool requestFromSlave(uint8_t quantity); 

    /**
     * @brief Write data as a slave.
     * @param data The data byte to send.
     * @return true if succeeded.
     */
    bool writeSlave(uint8_t data);

    /**
     * @brief Write multiple bytes as a slave.
     * @param data Pointer to the data to send.
     * @param length Number of bytes to send.
     * @return true if succeeded.
     */
    bool writeSlave(const uint8_t* data, size_t length);

    /**
     * @brief Read data as a slave.
     * @return The byte of data read, or -1 if no data available.
     */
    int readSlave();

    #endif                       

private:

    /**
     * @brief HAL I2C handle
     */
    I2C_HandleTypeDef *_hi2c;        

    /// @brief Receive buffer   
    uint8_t _rxBuffer[WIRE_BUFFER_LENGTH];   

    /// @brief Transmit buffer   
    uint8_t _txBuffer[WIRE_BUFFER_LENGTH];   

    /// @brief Current index in the receive buffer     
    uint8_t _rxIndex;   

    /// @brief Current index in the send buffer 
    uint8_t _txIndex;    

    /// @brief Length of received data                 
    uint8_t _rxLength; 

    /// @brief Length of send data  
    uint8_t _txLength;        

    /// @brief Address of the target device              
    uint8_t _txAddress;      

    /// @brief Transmission status flag
    uint8_t _transmitting;     


    #ifdef UNDER_DEVELOP
        /**
         * @brief Slave address for I2C communication.
         */
        uint8_t _slaveAddress;
        // Add flags for slave interrupts
        uint8_t _slaveRxCompleteFlag;
        uint8_t _slaveTxCompleteFlag;
        uint8_t _slaveBuffer[WIRE_BUFFER_LENGTH]; // Slave data buffer

        // Interrupt flags
        volatile uint8_t _txCompleteFlag; ///< TX complete flag
        volatile uint8_t _rxCompleteFlag; ///< RX complete flag

        void enableI2CInterrupts(); ///< Enable I2C interrupts
        void disableI2CInterrupts(); ///< Disable I2C interrupts
    #endif

    /// @brief Clears the send and receive buffers 
    void clearBuffers();         
};

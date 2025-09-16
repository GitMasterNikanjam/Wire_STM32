#pragma once

// ###########################################################################################

/**
 * @file Wire.h
 * @brief High-level I²C (TwoWire) wrapper for STM32 HAL (F1/F4/H7).
 *
 * @details
 * This header declares the `TwoWire` class: a small, portable wrapper around STM32 HAL I²C
 * that offers an Arduino-like API (`begin()`, `beginTransmission()`, `write()`, `endTransmission()`,
 * `requestFrom()`, `available()`, `read()`) for master mode, plus a minimal slave-mode interface.
 *
 * It supports three transfer modes:
 * - @ref WIRE_MODE_BLOCK (polling)
 * - @ref WIRE_MODE_INTERRUPT (IT, non-blocking)
 * - @ref WIRE_MODE_DMA (DMA, non-blocking)
 *
 * @note Provide a companion `mcu_select.h` that defines exactly one of STM32F1 / STM32F4 / STM32H7.
 * @see busRecoveryGPIO() for bus-unlock procedure via manual SCL clocking and STOP generation.
 */

// ===============================================================================
// MCU Selection Configuration
// ===============================================================================

#include "mcu_select.h"

/*
   If `mcu_select.h` does not exist yet, create it beside this file with content like:

   // mcu_select.h
   // #define STM32F1
   // #define STM32F4
   // #define STM32H7

   Uncomment exactly one macro for your target MCU family.
*/

// ###############################################################################################
// ========================================================================
// Include libraries:
// ===============================================================================

#if defined(STM32F1)
    #include "stm32f1xx_hal.h"          ///< HAL driver for STM32F1 series
#elif defined(STM32F4)
    #include "stm32f4xx_hal.h"          ///< HAL driver for STM32F4 series
#elif defined(STM32H7)
    #include "stm32h7xx_hal.h"          ///< HAL driver for STM32H7 series
#else
    #error "Unsupported MCU family. Please define STM32F1, STM32F4, or STM32H7 in mcu_select.h."
#endif

#include <string>                   ///< Required for error messages and string processing

// ###############################################################################################
// ===============================================================================
// Global Macro Definitions
// ===============================================================================

#ifndef WIRE_BUFFER_LENGTH
    /**
     * @brief Buffer size used for I2C transmit and receive operations.
     */
    #define WIRE_BUFFER_LENGTH      32  
#endif

#ifndef WIRE_MODE_BLOCK
    /**
     * @brief I2C communication mode: blocking (polling) mode.
     */
    #define WIRE_MODE_BLOCK         0
#endif

#ifndef WIRE_MODE_INTERRUPT
    /**
     * @brief I2C communication mode: interrupt-based communication.
     */
    #define WIRE_MODE_INTERRUPT     1
#endif

#ifndef WIRE_MODE_DMA
    /**
     * @brief I2C communication mode: DMA-based communication.
     */
    #define WIRE_MODE_DMA		    2
#endif 

// #define UNDER_DEVELOP  ///< Uncomment to enable experimental features

// ##############################################################################################
// TwoWire class:

/**
 * @class TwoWire
 * @brief High-level I²C communication wrapper on top of STM32 HAL.
 *
 * @par Features
 * - Arduino-like master API (begin/beginTransmission/write/endTransmission/requestFrom/available/read)
 * - Optional slave helpers (requestFromSlave / writeSlave / readSlave)
 * - Mode selection: blocking, interrupt, or DMA
 * - Simple timeout handling and bus recovery helpers
 *
 * @par Error Model
 * - `endTransmission()` returns Arduino-style codes (0..5)
 * - `errorCode` (public) is set by several methods for simple diagnostics
 *
 * @note Default TX/RX communication mode is @ref WIRE_MODE_BLOCK (polling).
 * @note For interrupt/DMA modes, call the corresponding `HAL_I2C_*CpltCallback()` from your global
 *       HAL callbacks, and forward into `TwoWire::*CpltCallback()` to update flags.
 * @warning If you plan to use @ref busRecoveryGPIO(), you must first call @ref setGPIO() to supply
 *          SCL/SDA ports/pins. Otherwise recovery will return false.
 */
class TwoWire 
{
    public:

        /**
         * @brief Last error code recorded during I2C communication.
         */
        uint8_t errorCode;

        /**
         * @brief Constructor.
         *
         * Initializes the TwoWire instance with a given I2C handle.
         *
         * @param i2cHandle Pointer to a pre-configured HAL I2C handle.
         *
         * @note It is recommended that:
         * - The I2C peripheral (I2C1, I2C2, etc.) be initialized before calling this constructor.
         * - GPIO pins for the I2C interface be configured before use.
         */
        TwoWire(I2C_HandleTypeDef *i2cHandle);
    
        /**
         * @brief Initializes the I2C interface in master mode.
         *
         * This method connects the controller (master) to the I2C bus.
         * It should normally be called once during setup.
         *
         * @return true if initialization succeeded, false otherwise.
         * @note - errorCode=1: The I2C handle instance is null.
         * @note - errorCode=2: The recovery() is not succeeded.
         */
        bool begin();     

        /**
         * @brief Initializes the I2C interface in slave mode.
         *
         * Joins the I2C bus using the given 7-bit address. If no address is specified,
         * it acts as a controller (master).
         *
         * @param address 7-bit I2C address to use in slave mode.
         * @return true if initialization succeeded, false otherwise.
         */
        bool begin(uint8_t address);       

        /**
         * @brief Disables the I2C interface and releases associated resources.
         *
         * Stops the I2C operations and resets the internal state.
         *
         * @return true if deinitialization succeeded, false otherwise.
         */
        bool end(void);

        /**
         * @brief Configure I²C bus speed.
         * @param clock Desired bus speed in Hz (e.g., 100000, 400000).
         * @return true if accepted and applied for the selected family, else false.
         *
         * @warning Call this **before** @ref begin(). H7 parts use `Init.Timing`; F1/F4 use `Init.ClockSpeed`.
         * @note For H7, example timings are provided; compute exact timings for your clock tree when possible.
         */
        bool setClock(uint32_t clock);

        /**
         * @brief Set I2C transmit mode that can be Block mode, Interrupt mode, DMA mode.
         * @param mode: Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         * @warning TxMode should be set before calling the begin() method.
         */
        bool setTxMode(uint8_t mode);

        /**
         * @brief Set I2C receive mode that can be Block mode, Interrupt mode, DMA mode.
         * @param mode: Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         * @warning RxMode should be set before calling the begin() method.
         */
        bool setRxMode(uint8_t mode);

        /**
         * @brief Sets the timeout for Wire transmissions in master mode.
         * @param timeout a timeout: timeout in milliseconds, if zero then timeout checking is disabled
         * @param resetWithTimeout: if true then Wire hardware will be automatically reset on timeout
         * @note these timeouts are almost always an indication of an underlying problem, such as misbehaving devices, noise, 
         * insufficient shielding, or other electrical problems. These timeouts will prevent your sketch from locking up, 
         * but not solve these problems. In such situations there will often (also) be data corruption which doesn’t result 
         * in a timeout or other error and remains undetected. So when a timeout happens, 
         * it is likely that some data previously read or written is also corrupted. 
         * Additional measures might be needed to more reliably detect such issues (e.g. checksums or reading back written values) 
         * and recover from them (e.g. full system reset). This timeout and such additional measures should be seen as a last line 
         * of defence, when possible the underlying cause should be fixed instead.
         */
        void setWireTimeout(uint32_t timeout, bool resetWithTimeout = false);

        /**
         * @brief Get wire timeout flag.
         */
        bool getWireTimeoutFlag(void);

        /**
         * @brief This function begins a transmission to the I2C peripheral device with the given address. 
         * Subsequently, queue bytes for transmission with the write() function and transmit them by calling endTransmission().
         * @param address: the 7-bit address of the device to transmit to.
         * @return true if succeeded.
         * @return False if the previous transmission has not completed yet.
         *  */                 
        bool beginTransmission(uint8_t address); 

        /**
         * @brief Write a byte.
         * @param data: a value to send as a single byte.
         * @note This function writes data from a peripheral device in response to a request from a controller device, 
         * or queues bytes for transmission from a controller to peripheral device (in-between calls to beginTransmission() and endTransmission()).
         * @return The number of bytes written (reading this number is optional).
         */
        uint8_t write(uint8_t data);              

        /**
         * @brief Write multiple bytes
         * @param data: a value to send as a single byte.
         * @param quantity: the number of bytes to transmit.
         * @note This function writes data from a peripheral device in response to a request from a controller device, 
         * or queues bytes for transmission from a controller to peripheral device (in-between calls to beginTransmission() and endTransmission()).
         * @return The number of bytes written (reading this number is optional).
         */
        uint8_t write(const uint8_t* data, uint8_t quantity); 

        /**
         * @brief Write a block of data to an 8-bit register on a 7-bit I²C device.
         *
         * @param dev7  7-bit device address (unshifted, e.g., 0x68).
         * @param reg   8-bit register/sub-address on the device.
         * @param data  Pointer to the buffer containing bytes to write.
         * @param len   Number of bytes to write from @p data.
         * @return HAL status code from @c HAL_I2C_Mem_Write().
         *
         * @pre The I²C peripheral must be initialized and enabled (see @ref begin()).
         * @pre @p data must be a valid pointer to at least @p len bytes.
         * @note This API issues a START, sends the device address and 8-bit register,
         *       writes @p len bytes, then issues a STOP. For many sensors, this is the
         *       standard “register write” sequence.
         * @warning On STM32H7 parts, correct @c Init.Timing must be set (see @ref setClock()).
         * @sa writeReg16(), readReg8(), readReg16()
         * 
         * @code
         * uint8_t bytes[] = {0x12, 0x34};
         * HAL_StatusTypeDef st = wire.writeReg8(0x68, 0x10, bytes, sizeof(bytes));
         * if (st != HAL_OK) { / handle error / }
         * @endcode
         *
        */
        HAL_StatusTypeDef writeReg8(uint8_t dev7, uint8_t reg, const uint8_t* data, size_t len);

        /**
         * @brief Read a block of data from an 8-bit register on a 7-bit I²C device.
         *
         * @param dev7  7-bit device address (unshifted).
         * @param reg   8-bit register/sub-address to read from.
         * @param data  Pointer to the buffer that will receive the bytes.
         * @param len   Number of bytes to read into @p data.
         * @return HAL status code from @c HAL_I2C_Mem_Read().
         *
         * @pre The I²C peripheral must be initialized and enabled (see @ref begin()).
         * @pre @p data must be a valid pointer to at least @p len bytes.
         * @note This API performs the common “register read” sequence:
         *       START → DevAddr(W) → Reg → (repeated) START → DevAddr(R) → read @p len → STOP.
         * @warning Some devices require delays after writing the register index; consult the datasheet.
         * @sa readReg16(), writeReg8(), writeReg16()
         *
         * @code
         * uint8_t rx[6];
         * if (wire.readReg8(0x68, 0x3B, rx, sizeof(rx)) == HAL_OK) {
         *   // rx now holds 6 bytes starting at register 0x3B
         * }
         * @endcode
         */
        HAL_StatusTypeDef readReg8(uint8_t dev7, uint8_t reg, uint8_t* data, size_t len);

        /**
         * @brief Write a block of data to a 16-bit register on a 7-bit I²C device.
         *
         * @param dev7  7-bit device address (unshifted).
         * @param reg   16-bit register/sub-address on the device.
         * @param data  Pointer to the buffer containing bytes to write.
         * @param len   Number of bytes to write from @p data.
         * @return HAL status code from @c HAL_I2C_Mem_Write().
         *
         * @pre The I²C peripheral must be initialized and enabled (see @ref begin()).
         * @pre @p data must be a valid pointer to at least @p len bytes.
         * @note The register address is sent MSB first, per STM32 HAL convention for 16-bit
         *       memory addressing (most devices expect this, but verify your datasheet).
         * @sa writeReg8(), readReg16(), readReg8()
         *
         * @code
         * uint8_t payload[4] = {1,2,3,4};
         * HAL_StatusTypeDef st = wire.writeReg16(0x50, 0x0123, payload, sizeof(payload));
         * @endcode
         */
        HAL_StatusTypeDef writeReg16(uint8_t dev7, uint16_t reg, const uint8_t* data, size_t len);

        /**
         * @brief Read a block of data from a 16-bit register on a 7-bit I²C device.
         *
         * @param dev7  7-bit device address (unshifted).
         * @param reg   16-bit register/sub-address to read from.
         * @param data  Pointer to the buffer that will receive the bytes.
         * @param len   Number of bytes to read into @p data.
         * @return HAL status code from @c HAL_I2C_Mem_Read().
         *
         * @pre The I²C peripheral must be initialized and enabled (see @ref begin()).
         * @pre @p data must be a valid pointer to at least @p len bytes.
         * @note Uses STM32 HAL “memory read” with 16-bit addressing; register MSB is sent first.
         * @sa readReg8(), writeReg16(), writeReg8()
         *
         * @code
         * uint8_t buf[2];
         * if (wire.readReg16(0x50, 0x0010, buf, sizeof(buf)) == HAL_OK) {
         *   // buf contains 2 bytes read from register 0x0010
         * }
         * @endcode
         */
        HAL_StatusTypeDef readReg16(uint8_t dev7, uint16_t reg, uint8_t* data, size_t len);

        /**
         * @brief This function ends a transmission to a peripheral device that was begun by beginTransmission() 
         * and transmits the bytes that were queued by write().
         * @return 
         * 0: success.
         * 
         * 1: data too long to fit in transmit buffer.
         * 
         * 2: received NACK on transmit of address.
         * 
         * 3: received NACK on transmit of data.
         * 
         * 4: other error.
         * 
         * 5: timeout
         */
        uint8_t endTransmission();               

        /**
         * @brief Request data from an I2C device.
         * @param address: the 7-bit slave address of the device to request bytes from.
         * @param quantity: the number of bytes to request.
         * @note This function is used by the controller device to request bytes from a peripheral device. 
         * The bytes may then be retrieved with the available() and read() functions. 
         * @return uint8_t: the number of bytes returned from the peripheral device.
         */
        uint8_t requestFrom(uint8_t address, uint8_t quantity); 

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

        /**
         * @brief Deinit and init i2c phrepheral for recovery i2c if nedded.
         */
        bool recovery(void);  
        
        /**
         * @brief Recovers the I2C bus by manually toggling the SCL line and generating a STOP condition.
         *
         * This function is used when the I2C bus becomes stuck, for example, if a slave device
         * (e.g., an EEPROM) holds the SDA line low due to an incomplete transaction or error.
         * It temporarily reconfigures the SCL and SDA pins as GPIO open-drain outputs, sends
         * up to 9 clock pulses on the SCL line to release the slave, and then generates a STOP
         * condition manually. Finally, it reinitializes the I2C peripheral.
         *
         * @note This function should be called when I2C communication fails due to a bus hang.
         *       It may be combined with additional recovery logic or delays depending on the
         *       specific slave devices used.
         *
         * @retval true  Bus recovery succeeded and I2C peripheral was reinitialized.
         * @retval false Recovery failed or I2C peripheral reinitialization error occurred.
         */
        bool busRecoveryGPIO(void);

        /**
         * @brief Clear timeout flag.
         */
        void clearWireTimeoutFlag(void);

        /**
         * @brief Interrupt callback function for masterTxCpltCallback
         * @note Use this function inside global I2C masterTxCpltCallback
         */
        void masterTxCpltCallback(void);

        /**
         * @brief Interrupt callback function for masterRxCpltCallback
         * @note Use this function inside global I2C masterRxCpltCallback
         */
        void masterRxCpltCallback(void);

        /**
         * @brief Interrupt callback function for slaveTxCpltCallback
         * @note Use this function inside global I2C slaveTxCpltCallback
         */
        void slaveTxCpltCallback(void);

        /**
         * @brief Interrupt callback function for slaveRxCpltCallback
         * @note Use this function inside global I2C slaveRxCpltCallback
         */
        void slaveRxCpltCallback(void);

        /**
         * @brief Interrupt callback function for slaveAddrCallback
         * @note Use this function inside global I2C slaveAddrCallback
         */
        void slaveAddrCallback(void);

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

        /// @brief Return interrupt txCompleteFlag
        bool getTxCompleteFlag(void) { return _txCompleteFlag;};

        /// @brief Return interrupt rxCompleteFlag
        bool getRxCompleteFlag(void) { return _rxCompleteFlag;};

        /// @brief Return interrupt slaveTxCompleteFlag
        bool getSlaveTxCompleteFlag(void) { return _slaveTxCompleteFlag;};

        /// @brief Return interrupt slaveRxCompleteFlag
        bool getSlaveRxCompleteFlag(void) { return _slaveRxCompleteFlag;};

        /**
         * @brief This function is used to check if a slave device is available on the I2C bus before initiating communication. 
         * This helps prevent HAL_BUSY errors or failed transactions.
         */
        bool IsDeviceReady(uint8_t address);

        /**
         * @brief Check I2C State.
         */
        HAL_I2C_StateTypeDef GetState(void);

        /**
         * @brief Set GPIO of SCL and SDA.
         */
        void setGPIO(GPIO_TypeDef* SCLPort, uint16_t SCLPin, GPIO_TypeDef* SDAPort, uint16_t SDAPIn);

        /**
         * @brief ACK polling: repeatedly call HAL_IsDeviceReady() until success or timeout.
         * @param dev7bit 7-bit device address.
         * @param timeout_ms Maximum wait in milliseconds.
         * @return HAL_OK on success; HAL_TIMEOUT on timeout.
         */
        HAL_StatusTypeDef ackPoll(uint16_t dev7bit, uint32_t timeout_ms);

    private:

        // --------- Recovery GPIO (optional) ---------

        GPIO_TypeDef* _SCL_PORT;      ///< SCL port for manual recovery
        uint16_t      _SCL_PIN;       ///< SCL pin mask for manual recovery
        GPIO_TypeDef* _SDA_PORT;      ///< SDA port for manual recovery
        uint16_t      _SDA_PIN;       ///< SDA pin mask for manual recovery

        // --------- Core state ---------

        /**
         * @brief HAL I2C handle pointer
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

        /// @brief Transmission status flag
        uint8_t _transmitting;     

        /// @brief The flag that enable/disable recovery i2c if timeout occured. A value of true means i2c recovery after timeout occurred.
        bool _resetWithTimeout;

        /// @brief The timeout value for i2c communication. [ms]
        uint32_t _timeout;

        /// @brief The timeout event flag.
        bool _timeoutFlag;

        /// @brief Address of the target device              
        uint8_t _txAddress;

        /**
         * @brief Slave address for I2C communication.
         */
        uint8_t _slaveAddress;

        /// @brief The flag for TX master interrupt
        volatile bool _txCompleteFlag;

        /// @brief The flag for RX master interrupt
        volatile bool _rxCompleteFlag; 

        /// @brief The flag for RX slave interrupt
        volatile bool _slaveRxCompleteFlag;

        /// @brief The flag for TX slave interrupt
        volatile bool _slaveTxCompleteFlag;

        /// @brief Slave data buffer
        uint8_t _slaveBuffer[WIRE_BUFFER_LENGTH]; 

        /**
         * @brief I2C transmit mode that can be Block mode, Interrupt mode, DMA mode.
         * @note - Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         */
        volatile uint8_t _txMode;

        /**
         * @brief I2C receive mode that can be Block mode, Interrupt mode, DMA mode.
         * @note - Can be 0: WIRE_MODE_BLOCK, 1: WIRE_MODE_INTERRUPT, 3: WIRE_MODE_DMA.
         * @return true if succeeded.
         */
        volatile uint8_t _rxMode;

        /// @brief Clears the send and receive buffers 
        void _clearBuffers(); 

};





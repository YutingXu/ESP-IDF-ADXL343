/**************************************************************************/
/*!
		@file     ADXL343.h
		@author   Bryan Siepert and K. Townsend (Adafruit Industries)

		Ported to ESP-IDF by Yuting Xu
		
		BSD license (see license.txt)

		This is a library for the Adafruit ADS1015 breakout board
		----> https://www.adafruit.com/products/???

		Adafruit invests time and resources providing this open source code,
		please support Adafruit and open-source hardware by purchasing
		products from Adafruit!

		v1.0  - First release
*/
/**************************************************************************/
#ifndef ADXL343_H
#define ADXL343_H

#include "driver/spi_master.h"
// #include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_types.h"
#include "string.h"

/*=========================================================================
		I2C ADDRESS/BITS
		-----------------------------------------------------------------------*/
#define ADXL343_ADDRESS              (0x53) /**< Assumes ALT address pin low */
/*=========================================================================*/

/*=========================================================================
		REGISTERS
		-----------------------------------------------------------------------*/
#define ADXL3XX_REG_DEVID            (0x00)        /**< Device ID */
#define ADXL3XX_REG_THRESH_TAP       (0x1D)        /**< Tap threshold */
#define ADXL3XX_REG_OFSX             (0x1E)        /**< X-axis offset */
#define ADXL3XX_REG_OFSY             (0x1F)        /**< Y-axis offset */
#define ADXL3XX_REG_OFSZ             (0x20)        /**< Z-axis offset */
#define ADXL3XX_REG_DUR              (0x21)        /**< Tap duration */
#define ADXL3XX_REG_LATENT           (0x22)        /**< Tap latency */
#define ADXL3XX_REG_WINDOW           (0x23)        /**< Tap window */
#define ADXL3XX_REG_THRESH_ACT       (0x24)        /**< Activity threshold */
#define ADXL3XX_REG_THRESH_INACT     (0x25)        /**< Inactivity threshold */
#define ADXL3XX_REG_TIME_INACT       (0x26)        /**< Inactivity time */
#define ADXL3XX_REG_ACT_INACT_CTL    (0x27)        /**< Axis enable control for activity and inactivity detection */
#define ADXL3XX_REG_THRESH_FF        (0x28)        /**< Free-fall threshold */
#define ADXL3XX_REG_TIME_FF          (0x29)        /**< Free-fall time */
#define ADXL3XX_REG_TAP_AXES         (0x2A)        /**< Axis control for single/double tap */
#define ADXL3XX_REG_ACT_TAP_STATUS   (0x2B)        /**< Source for single/double tap */
#define ADXL3XX_REG_BW_RATE          (0x2C)        /**< Data rate and power mode control */
#define ADXL3XX_REG_POWER_CTL        (0x2D)        /**< Power-saving features control */
#define ADXL3XX_REG_INT_ENABLE       (0x2E)        /**< Interrupt enable control */
#define ADXL3XX_REG_INT_MAP          (0x2F)        /**< Interrupt mapping control */
#define ADXL3XX_REG_INT_SOURCE       (0x30)        /**< Source of interrupts */
#define ADXL3XX_REG_DATA_FORMAT      (0x31)        /**< Data format control */
#define ADXL3XX_REG_DATAX0           (0x32)        /**< X-axis data 0 */
#define ADXL3XX_REG_DATAX1           (0x33)        /**< X-axis data 1 */
#define ADXL3XX_REG_DATAY0           (0x34)        /**< Y-axis data 0 */
#define ADXL3XX_REG_DATAY1           (0x35)        /**< Y-axis data 1 */
#define ADXL3XX_REG_DATAZ0           (0x36)        /**< Z-axis data 0 */
#define ADXL3XX_REG_DATAZ1           (0x37)        /**< Z-axis data 1 */
#define ADXL3XX_REG_FIFO_CTL         (0x38)        /**< FIFO control */
#define ADXL3XX_REG_FIFO_STATUS      (0x39)        /**< FIFO status */
/*=========================================================================*/

/*=========================================================================
		REGISTERS
		-----------------------------------------------------------------------*/
#define ADXL343_MG2G_MULTIPLIER      (0.004)       /**< 4mg per lsb */
/*=========================================================================*/

#define ADXL343_GRAVITY (9.80665F) /**< Earth's gravity in m/s^2 */

typedef struct ADXL343RegisterMap
{
	uint8_t power_control; // internal copy of 0x2D
	uint8_t data_format;
} ADXL343RegisterMap;

/** Used with register 0x2C (ADXL3XX_REG_BW_RATE) to set bandwidth */
typedef enum {
	ADXL343_DATARATE_3200_HZ = 0b1111, /**< 3200Hz Bandwidth */
	ADXL343_DATARATE_1600_HZ = 0b1110, /**< 1600Hz Bandwidth */
	ADXL343_DATARATE_800_HZ = 0b1101,  /**<  800Hz Bandwidth */
	ADXL343_DATARATE_400_HZ = 0b1100,  /**<  400Hz Bandwidth */
	ADXL343_DATARATE_200_HZ = 0b1011,  /**<  200Hz Bandwidth */
	ADXL343_DATARATE_100_HZ = 0b1010,  /**<  100Hz Bandwidth */
	ADXL343_DATARATE_50_HZ = 0b1001,   /**<   50Hz Bandwidth */
	ADXL343_DATARATE_25_HZ = 0b1000,   /**<   25Hz Bandwidth */
	ADXL343_DATARATE_12_5_HZ = 0b0111, /**< 12.5Hz Bandwidth */
	ADXL343_DATARATE_6_25HZ = 0b0110,  /**< 6.25Hz Bandwidth */
	ADXL343_DATARATE_3_13_HZ = 0b0101, /**< 3.13Hz Bandwidth */
	ADXL343_DATARATE_1_56_HZ = 0b0100, /**< 1.56Hz Bandwidth */
	ADXL343_DATARATE_0_78_HZ = 0b0011, /**< 0.78Hz Bandwidth */
	ADXL343_DATARATE_0_39_HZ = 0b0010, /**< 0.39Hz Bandwidth */
	ADXL343_DATARATE_0_20_HZ = 0b0001, /**< 0.20Hz Bandwidth */
	ADXL343_DATARATE_0_10_HZ = 0b0000, /**< 0.10Hz Bandwidth (default value) */

	ADXL3XX_DATARATE_3200_HZ = 0b1111, /**< 3200Hz Bandwidth */
	ADXL3XX_DATARATE_1600_HZ = 0b1110, /**< 1600Hz Bandwidth */
	ADXL3XX_DATARATE_800_HZ = 0b1101,  /**<  800Hz Bandwidth */
	ADXL3XX_DATARATE_400_HZ = 0b1100,  /**<  400Hz Bandwidth */
	ADXL3XX_DATARATE_200_HZ = 0b1011,  /**<  200Hz Bandwidth */
	ADXL3XX_DATARATE_100_HZ = 0b1010,  /**<  100Hz Bandwidth */
	ADXL3XX_DATARATE_50_HZ = 0b1001,   /**<   50Hz Bandwidth */
	ADXL3XX_DATARATE_25_HZ = 0b1000,   /**<   25Hz Bandwidth */
	ADXL3XX_DATARATE_12_5_HZ = 0b0111, /**< 12.5Hz Bandwidth */
	ADXL3XX_DATARATE_6_25HZ = 0b0110,  /**< 6.25Hz Bandwidth */
	ADXL3XX_DATARATE_3_13_HZ = 0b0101, /**< 3.13Hz Bandwidth */
	ADXL3XX_DATARATE_1_56_HZ = 0b0100, /**< 1.56Hz Bandwidth */
	ADXL3XX_DATARATE_0_78_HZ = 0b0011, /**< 0.78Hz Bandwidth */
	ADXL3XX_DATARATE_0_39_HZ = 0b0010, /**< 0.39Hz Bandwidth */
	ADXL3XX_DATARATE_0_20_HZ = 0b0001, /**< 0.20Hz Bandwidth */
	ADXL3XX_DATARATE_0_10_HZ = 0b0000  /**< 0.10Hz Bandwidth (default value) */
} ADXL343DataRate;

/** Used with register 0x31 (ADXL3XX_REG_DATA_FORMAT) to set g range */
typedef enum {
	ADXL343_RANGE_2_G = 0b00,  /**< +/- 2g (default value) */
	ADXL343_RANGE_4_G = 0b01,  /**< +/- 4g */
	ADXL343_RANGE_8_G = 0b10,  /**< +/- 8g */
	ADXL343_RANGE_16_G = 0b11 /**< +/- 16g */
} ADXL343Range;

/** Possible interrupts sources on the ADXL343. */
typedef union {
	uint8_t value; /**< Composite 8-bit value of the bitfield.*/
	struct {
		uint8_t overrun : 1;    /**< Bit 0 */
		uint8_t watermark : 1;  /**< Bit 1 */
		uint8_t freefall : 1;   /**< Bit 2 */
		uint8_t inactivity : 1; /**< Bit 3 */
		uint8_t activity : 1;   /**< Bit 4 */
		uint8_t double_tap : 1; /**< Bit 5 */
		uint8_t single_tap : 1; /**< Bit 6 */
		uint8_t data_ready : 1; /**< Bit 7 */
	} bits;                   /**< Individual bits in the bitfield. */
} ADXL343IntrConfig;

/** Possible interrupt pin outputs on the ADXL343. */
typedef enum {
	ADXL343_INT1 = 0,
	ADXL343_INT2 = 1,
} ADXL343IntrPin;

typedef union {
	uint8_t value; /**< Composite 8-bit value of the bitfield.*/
	struct {
		ADXL343IntrPin overrun : 1;    /**< Bit 0 */
		ADXL343IntrPin watermark : 1;  /**< Bit 1 */
		ADXL343IntrPin freefall : 1;   /**< Bit 2 */
		ADXL343IntrPin inactivity : 1; /**< Bit 3 */
		ADXL343IntrPin activity : 1;   /**< Bit 4 */
		ADXL343IntrPin double_tap : 1; /**< Bit 5 */
		ADXL343IntrPin single_tap : 1; /**< Bit 6 */
		ADXL343IntrPin data_ready : 1; /**< Bit 7 */
	} bits;                   /**< Individual bits in the bitfield. */
} ADXL343IntrMapping;

typedef enum {
	ADXL343_SPI = 0,
	ADXL343_I2C = 1
} ADXL343InterfaceType;

typedef struct ADXL343I2CSettings
{
	uint8_t scl_pin;
	uint8_t sda_pin;
	uint8_t addr; // 7-bit I2C addresss
	uint8_t port_num;
	uint32_t freq;
} ADXL343I2CSettings;

typedef struct ADXL343SPISettings
{
	uint8_t mosi_pin;
	uint8_t miso_pin;
	uint8_t sclk_pin;
	uint8_t cs_pin;
	uint8_t bus_num;
	uint32_t freq;
	spi_device_handle_t _spi_handle;
} ADXL343SPISettings;

typedef struct ADXL343InterfaceSettings
{
	ADXL343InterfaceType interface_type;
	union
	{
		ADXL343I2CSettings i2c;
		ADXL343SPISettings spi;
	} interface;
} ADXL343InterfaceSettings;

typedef struct ADXL343_cfg
{
	ADXL343InterfaceSettings interface;
	uint32_t sensor_id;
	ADXL343Range _range;
	ADXL343RegisterMap _registers;
} ADXL343_cfg;


/**************************************************************************/
/*!
		@brief  Writes 8-bits to the specified destination register

		@param reg The register to write to
		@param value The value to write to the register
*/
/**************************************************************************/
void ADXL343_write_reg(ADXL343_cfg *pcfg, uint8_t addr, uint8_t reg);

/**************************************************************************/
/*!
		@brief  Reads 8-bits from the specified register

		@param reg register to read

		@return The results of the register read request
*/
/**************************************************************************/
uint8_t ADXL343_read_reg(ADXL343_cfg *pcfg, uint8_t addr);

/**************************************************************************/
/*!
		@brief  Reads 16-bits from the specified register

		@param reg The register to read two bytes from

		@return The 16-bit value read from the reg starting address
*/
/**************************************************************************/
uint16_t ADXL343_read_reg16(ADXL343_cfg *pcfg, uint8_t addr);

/**************************************************************************/
/*!
		@brief  Read the device ID (can be used to check connection)

		@return The 8-bit device ID
*/
/**************************************************************************/
uint8_t ADXL343_get_device_id(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Enables (1) or disables (0) the interrupts on the specified
						interrupt pin.

		@param cfg The bitfield of the interrupts to enable or disable.

		@return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
void ADXL343_enable_interrupts(ADXL343_cfg *pcfg, ADXL343IntrConfig intr_cfg);

/**************************************************************************/
/*!
		@brief  'Maps' the specific interrupt to either pin INT1 (bit=0),
						of pin INT2 (bit=1).

		@param cfg The bitfield of the interrupts to enable or disable.

		@return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
void ADXL343_map_interrupts(ADXL343_cfg *pcfg, ADXL343IntrMapping intr_pin);

/**************************************************************************/
/*!
		@brief  Sets the data rate for the ADXL343 (controls power consumption)

		@param dataRate The data rate to set, based on adxl3xx_dataRate_t
*/
/**************************************************************************/
void ADXL343_set_datarate(ADXL343_cfg *pcfg, ADXL343DataRate data_rate);


ADXL343DataRate ADXL343_get_datarate(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Setups the HW (reads coefficients values, etc.)
		@param  i2caddr The 7-bit I2C address to find the ADXL on
		@return True if the sensor was successfully initialised.
*/
/**************************************************************************/
uint8_t ADXL343_initialise(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Reads the status of the interrupt pins. Reading this register
						also clears or deasserts any currently active interrupt.

		@return The 8-bit content of the INT_SOURCE register.
*/
/**************************************************************************/
uint8_t ADXL343_check_interrupts(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Gets the most recent X axis value

		@return The 16-bit signed value for the X axis
*/
/**************************************************************************/
int16_t ADXL343_get_x(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Gets the most recent Y axis value

		@return The 16-bit signed value for the Y axis
*/
/**************************************************************************/
int16_t ADXL343_get_y(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Gets the most recent Z axis value

		@return The 16-bit signed value for the Z axis
*/
/**************************************************************************/
int16_t ADXL343_get_z(ADXL343_cfg *pcfg);

// /**************************************************************************/
// /*!
// 		@brief  Reads 3x16-bits from the x, y, and z data register
// 		@param x reference to return x acceleration data
// 		@param y reference to return y acceleration data
// 		@param z reference to return z acceleration data
// 		@return True if the operation was successful, otherwise false.
// */
// /**************************************************************************/
// void ADXL343_get_xyz(ADXL343_cfg *pcfg, int16_t *x, int16_t *y, int16_t *z);

/**************************************************************************/
/*!
		@brief  Sets the g range for the accelerometer

		@param range The range to set, based on adxl34x_range_t
*/
/**************************************************************************/
void ADXL343_set_range(ADXL343_cfg *pcfg, ADXL343Range range);


/**************************************************************************/
/*!
		@brief  Sets the g range for the accelerometer

		@return The adxl34x_range_t value corresponding to the sensors range
*/
/**************************************************************************/
ADXL343Range ADXL343_get_range(ADXL343_cfg *pcfg);

/**************************************************************************/
/*!
		@brief  Retrieves the X Y Z trim offsets, note that they are 4 bits signed
		but we use int8_t to store and 'extend' the sign bit!
		@param x Pointer to the x offset, from -5 to 4 (internally multiplied by 8
	 lsb)
		@param y Pointer to the y offset, from -5 to 4 (internally multiplied by 8
	 lsb)
		@param z Pointer to the z offset, from -5 to 4 (internally multiplied by 8
	 lsb)
*/
/**************************************************************************/
void ADXL343_get_trim_offsets(ADXL343_cfg *pcfg, int8_t *x, int8_t *y, int8_t *z);

/**************************************************************************/
/*!
		@brief  Sets the X Y Z trim offsets, note that they are 4 bits signed
		but we use int8_t to store and 'extend' the sign bit!
		@param x The x offset, from -5 to 4 (internally multiplied by 8 lsb)
		@param y The y offset, from -5 to 4 (internally multiplied by 8 lsb)
		@param z The z offset, from -5 to 4 (internally multiplied by 8 lsb)
*/
/**************************************************************************/
void ADXL343_set_trim_offsets(ADXL343_cfg *pcfg, int8_t x, int8_t y, int8_t z);

void ADXL343_get_x_y_z(ADXL343_cfg *pcfg, int16_t *x, int16_t *y, int16_t *z);

/**************************************************************************/
/*!
		@brief  Helper function to set bitfield of a 8-bit register
		@param preg Pointer to the register of interest
		@param start Starting position of bitfield (starting from 0)
		@param len Length of bitfield
		@param value The value to be set
*/
/**************************************************************************/
void ADXL343_set_bf(uint8_t *preg, uint8_t start, uint8_t len, uint8_t value);

/**************************************************************************/
/*!
		@brief  Helper function to get bitfield of a 8-bit register
		@param preg Pointer to the register of interest
		@param start Starting position of bitfield (starting from 0)
		@param len Length of bitfield
		@return Content in desired bitfield
*/
/**************************************************************************/
uint8_t ADXL343_get_bf(uint8_t *preg, uint8_t start, uint8_t len);

#endif
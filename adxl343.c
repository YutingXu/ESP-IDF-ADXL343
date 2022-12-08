/**************************************************************************/
/*!
		@file     Adafruit_ADXL343.c
		@author   Bryan Siepert and K.Townsend (Adafruit Industries)

		Ported to ESP-IDF by Yuting Xu

		BSD License (see license.txt)

		The ADXL343 is a digital accelerometer with 13-bit resolution, capable
		of measuring up to +/-16g.  This driver communicates using I2C.

		This is a library for the Adafruit ADXL343 breakout
		----> https://www.adafruit.com/product/4097
		or the Adafruit ADXL343 + ADT7410 FeatherWing
		----> https://www.adafruit.com/product/4147

		Adafruit invests time and resources providing this open source code,
		please support Adafruit and open-source hardware by purchasing
		products from Adafruit!

		v1.0 - First release
*/
/**************************************************************************/
#include "adxl343.h"

static const char *TAG = "ADXL343 lib";

#ifdef CONFIG_IDF_TARGET_ESP32
#define HOST1    HSPI_HOST
#define HOST2    VSPI_HOST

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define HOST1    SPI2_HOST
#define HOST2    SPI3_HOST

#elif defined CONFIG_IDF_TARGET_ESP32C3
#define HOST1    SPI2_HOST
#define HOST2    SPI3_HOST

#elif defined CONFIG_IDF_TARGET_ESP32S3
#define HOST1    SPI2_HOST
#define HOST2    SPI3_HOST
#endif

void ADXL343_write_reg(ADXL343_cfg *pcfg, uint8_t addr, uint8_t reg)
{
	esp_err_t ret;
	if(pcfg->interface.interface_type == ADXL343_I2C)
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, addr, true);
		i2c_master_write_byte(cmd, reg, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}
	else
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.tx_data[0] = reg;
		t.flags = SPI_TRANS_USE_TXDATA;
		t.length = 8;
		t.addr = addr;
		t.rx_buffer = NULL;

		spi_device_acquire_bus(pcfg->interface.interface.spi._spi_handle, portMAX_DELAY);
		ret = spi_device_transmit(pcfg->interface.interface.spi._spi_handle, &t);
		ESP_ERROR_CHECK(ret);
		spi_device_release_bus(pcfg->interface.interface.spi._spi_handle);
	}

}

uint8_t ADXL343_read_reg(ADXL343_cfg *pcfg, uint8_t addr)
{
	esp_err_t ret;
	if(pcfg->interface.interface_type == ADXL343_I2C)
	{
		uint8_t temp_reg = 0;

		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, addr, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_READ, true);
		i2c_master_read_byte(cmd, &temp_reg, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		return temp_reg;
	}
	else
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.addr = addr; // use address phase
		t.tx_buffer = NULL; // skip write phase
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 8;

		ret = spi_device_transmit(pcfg->interface.interface.spi._spi_handle, &t);
		ESP_ERROR_CHECK(ret);

		return t.rx_data[0];
	}
}

uint16_t ADXL343_read_reg16(ADXL343_cfg *pcfg, uint8_t addr)
{
	esp_err_t ret;
	if(pcfg->interface.interface_type == ADXL343_I2C)
	{
		uint8_t temp_reg[2] = {0, 0};

		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, addr, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_READ, true);
		i2c_master_read_byte(cmd, temp_reg, false);
		i2c_master_read_byte(cmd, temp_reg+1, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
		return temp_reg[1] << 8 | temp_reg[0];
	}
	else
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.addr = addr; // use address phase
		t.tx_buffer = NULL; // skip write phase
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 16;

		ret = spi_device_transmit(pcfg->interface.interface.spi._spi_handle, &t);
		ESP_ERROR_CHECK(ret);

		return t.rx_data[1] << 8 | t.rx_data[0];
	}
}

void ADXL343_get_x_y_z(ADXL343_cfg *pcfg, int16_t *x, int16_t *y, int16_t *z)
{
	esp_err_t ret;
	if(pcfg->interface.interface_type == ADXL343_I2C)
	{
		uint8_t temp_reg[6] = {0, 0, 0, 0, 0, 0};
		// ret = i2c_master_read_from_device(
		// 	pcfg->interface.interface.i2c.port_num, 
		// 	pcfg->interface.interface.i2c.addr,
		// 	temp_reg, 
		// 	sizeof(temp_reg),
		// 	1000 / portTICK_PERIOD_MS
		// );
		// ESP_ERROR_CHECK(ret);

		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, ADXL3XX_REG_DATAX0, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (pcfg->interface.interface.i2c.addr << 1) | I2C_MASTER_READ, true);
		i2c_master_read_byte(cmd, temp_reg, false);
		i2c_master_read_byte(cmd, temp_reg+1, false);
		i2c_master_read_byte(cmd, temp_reg+2, false);
		i2c_master_read_byte(cmd, temp_reg+3, false);
		i2c_master_read_byte(cmd, temp_reg+4, false);
		i2c_master_read_byte(cmd, temp_reg+5, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(pcfg->interface.interface.i2c.port_num, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
		*x = temp_reg[1] << 8 | temp_reg[0];
		*y = temp_reg[3] << 8 | temp_reg[2];
		*z = temp_reg[5] << 8 | temp_reg[4];
	}
	else
	{
		// TODO: implement SPI protocol

		// spi_transaction_t t;
		// memset(&t, 0, sizeof(t));
		// t.addr = addr; // use address phase
		// t.tx_buffer = NULL; // skip write phase;
		// t.length = 0;
		// t.rxlength = 48;

		// ret = spi_device_transmit(pcfg->interface.interface.spi._spi_handle, &t);
		// ESP_ERROR_CHECK(ret);

		// return t.rx_data[0] << 8 | t.rx_data[1];
	}
}


uint8_t ADXL343_get_device_id(ADXL343_cfg *pcfg)
{
	return ADXL343_read_reg(pcfg, ADXL3XX_REG_DEVID);
}

void ADXL343_enable_interrupts(ADXL343_cfg *pcfg, ADXL343IntrConfig intr_cfg)
{
	ADXL343_write_reg(pcfg, ADXL3XX_REG_INT_ENABLE, intr_cfg.value);
}

void ADXL343_map_interrupts(ADXL343_cfg *pcfg, ADXL343IntrMapping intr_pin)
{
	ADXL343_write_reg(pcfg, ADXL3XX_REG_INT_MAP, intr_pin.value);
}

uint8_t ADXL343_check_interrupts(ADXL343_cfg *pcfg)
{
	return ADXL343_read_reg(pcfg, ADXL3XX_REG_INT_SOURCE);
}

int16_t ADXL343_get_x(ADXL343_cfg *pcfg)
{
	return ((int16_t) ADXL343_read_reg16(pcfg, ADXL3XX_REG_DATAX0));
}

int16_t ADXL343_get_y(ADXL343_cfg *pcfg)
{
	return ((int16_t) ADXL343_read_reg16(pcfg, ADXL3XX_REG_DATAY0));
}

int16_t ADXL343_get_z(ADXL343_cfg *pcfg)
{
	return ((int16_t) ADXL343_read_reg16(pcfg, ADXL3XX_REG_DATAZ0));
}

uint8_t ADXL343_initialise(ADXL343_cfg *pcfg)
{
	if(pcfg->interface.interface_type == ADXL343_I2C)
	{
		ESP_LOGI(TAG, "Operating at frequency of %d", ets_get_cpu_frequency());
		ESP_LOGI(TAG, "Attempting to initialise I2C interface");
		ESP_LOGI(TAG, "SCL pin: %d, SDA pin: %d", pcfg->interface.interface.i2c.scl_pin, pcfg->interface.interface.i2c.sda_pin);
		i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = pcfg->interface.interface.i2c.sda_pin,
			.scl_io_num = pcfg->interface.interface.i2c.scl_pin,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = pcfg->interface.interface.i2c.freq,
		};

		i2c_param_config(pcfg->interface.interface.i2c.port_num, &conf);


		esp_err_t ret;
		ret = i2c_driver_install(pcfg->interface.interface.i2c.port_num, conf.mode, 0, 0, 0);
		ESP_ERROR_CHECK(ret);
		int sample_time, hold_time;
		i2c_set_data_timing(pcfg->interface.interface.i2c.port_num, 99, 20);
		i2c_get_data_timing(pcfg->interface.interface.i2c.port_num, &sample_time, &hold_time); // default 99, 99
		ESP_LOGI(TAG, "I2C interface successfully initialised");
		ESP_LOGI(TAG, "I2C data timing: sample timing %d ticks, hold time %d ticks", sample_time, hold_time);
		i2c_set_start_timing(pcfg->interface.interface.i2c.port_num, 199, 260);
		i2c_get_start_timing(pcfg->interface.interface.i2c.port_num, &sample_time, &hold_time); // default 199, 200
		ESP_LOGI(TAG, "I2C start timing: setup time %d ticks, hold time %d ticks", sample_time, hold_time);
	}
	else
	{
		ESP_LOGI(TAG, "Attempting to initialise SPI interface");
		spi_bus_config_t buscfg = 
		{
			.mosi_io_num = pcfg->interface.interface.spi.mosi_pin,
			.miso_io_num = pcfg->interface.interface.spi.miso_pin,
			.sclk_io_num = pcfg->interface.interface.spi.sclk_pin,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
		};

		int busno = HOST1; // default to SPI bus 2
		if (pcfg->interface.interface.spi.bus_num == 3)
		{
			busno = HOST1; // SPI bus 2
		}
		else if (pcfg->interface.interface.spi.bus_num == 4)
		{
			busno = HOST2; // SPI bus 3
		}
		else
		{
			ESP_LOGE(TAG, "Invalid SPI bus number");
			return 1;
		}

		esp_err_t ret;
		ret = spi_bus_initialize(busno, &buscfg, SPI_DMA_CH_AUTO);
		ESP_ERROR_CHECK(ret);

		spi_device_interface_config_t devcfg = 
		{
			.command_bits = 0,
			.address_bits = 8,
			.dummy_bits = 0,
			.clock_speed_hz = pcfg->interface.interface.spi.freq,
			.duty_cycle_pos = 128,        // 50% duty cycle
			.mode = 0,
			.spics_io_num = pcfg->interface.interface.spi.cs_pin, 
			.cs_ena_posttrans = 0,        
			.queue_size = 7
		};

		ret = spi_bus_add_device(busno, &devcfg, &pcfg->interface.interface.spi._spi_handle);
		ESP_ERROR_CHECK(ret);
		ESP_LOGI(TAG, "SPI interface successfully initialised");
	}

	/* Check connection */
	uint8_t deviceid = ADXL343_get_device_id(pcfg);
	if (deviceid != 0xE5) 
	{
		/* No ADXL343 detected ... return 1 */
		ESP_LOGE(TAG, "No ADXL343 detected!");
		return 1;
	}
	else
	{
		ESP_LOGI(TAG, "ADXL343 detected!");
	}

	pcfg->_range = ADXL343_RANGE_2_G;

	// 	// Default tap detection level (2G, 31.25ms duration, single tap only)
	// 	// If only the single tap function is in use, the single tap interrupt
	// 	// is triggered when the acceleration goes below the threshold, as
	// 	// long as DUR has not been exceeded.
	// ADXL343_write_reg(pcfg, ADXL3XX_REG_INT_ENABLE, 0);  // Disable interrupts to start
	// ADXL343_write_reg(pcfg, ADXL3XX_REG_THRESH_TAP, 20); // 62.5 mg/LSB (so 0xFF = 16 g)
	// ADXL343_write_reg(pcfg, ADXL3XX_REG_DUR, 50);        // Max tap duration, 625 µs/LSB
	// ADXL343_write_reg(pcfg, ADXL3XX_REG_LATENT, 0);      // Tap latency, 1.25 ms/LSB, 0=no double tap
	// ADXL343_write_reg(pcfg, ADXL3XX_REG_WINDOW, 0);      // Waiting period,  1.25 ms/LSB, 0=no double tap
	// ADXL343_write_reg(pcfg, ADXL3XX_REG_TAP_AXES, 0x7);  // Enable the XYZ axis for tap

	// Enable measurements
	ADXL343_write_reg(pcfg, ADXL3XX_REG_POWER_CTL, 0x08);

	return 0;
}

void ADXL343_set_range(ADXL343_cfg *pcfg, ADXL343Range range)
{
	ADXL343_set_bf(&pcfg->_registers.data_format, 0, 2, range);
	ADXL343_set_bf(&pcfg->_registers.data_format, 3, 1, 1);
	ADXL343_write_reg(pcfg, ADXL3XX_REG_DATA_FORMAT, pcfg->_registers.data_format);

	pcfg->_range = range;
}

ADXL343Range ADXL343_get_range(ADXL343_cfg *pcfg)
{
	return ADXL343_read_reg(pcfg, ADXL3XX_REG_DATA_FORMAT) & 0x03;
}

void ADXL343_set_datarate(ADXL343_cfg *pcfg, ADXL343DataRate data_rate)
{
	ADXL343_write_reg(pcfg, ADXL3XX_REG_BW_RATE, data_rate);
}

ADXL343DataRate ADXL343_get_datarate(ADXL343_cfg *pcfg)
{
	return ADXL343_read_reg(pcfg, ADXL3XX_REG_BW_RATE) & 0x0F;
}


void ADXL343_get_trim_offsets(ADXL343_cfg *pcfg, int8_t *x, int8_t *y, int8_t *z)
{
	if(x != NULL)
		*x = ADXL343_read_reg(pcfg, ADXL3XX_REG_OFSX);

	if(y != NULL)
		*y = ADXL343_read_reg(pcfg, ADXL3XX_REG_OFSY);

	if(z != NULL)
		*z = ADXL343_read_reg(pcfg, ADXL3XX_REG_OFSZ);
}

void ADXL343_set_trim_offsets(ADXL343_cfg *pcfg, int8_t x, int8_t y, int8_t z)
{

	ADXL343_write_reg(pcfg, ADXL3XX_REG_OFSX, x);
	ADXL343_write_reg(pcfg, ADXL3XX_REG_OFSY, y);
	ADXL343_write_reg(pcfg, ADXL3XX_REG_OFSZ, z);
}

void ADXL343_set_bf(uint8_t *preg, uint8_t start, uint8_t len, uint8_t value)
{
    uint8_t bitmask = (1 << len) - 1;
    value &= bitmask;
    bitmask <<= start;
    *preg = (*preg & (~bitmask)) | (value << start);
}

uint8_t ADXL343_get_bf(uint8_t *preg, uint8_t start, uint8_t len)
{
    uint8_t bitmask = ((1 << len) - 1) << start;
    uint8_t result = (*preg & bitmask) >> start;
    return result;
}


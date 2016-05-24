/**
 * @file ms5611.h
 *
 * Shared defines for the ms5611 driver.
 */

#define ADDR_RESET_CMD				0x1E	/* write to this address to reset chip */          //芯片复位
#define ADDR_CMD_CONVERT_D1		0x48	/* write to this address to start pressure conversion */   //开始压力转换
#define ADDR_CMD_CONVERT_D2		0x58	/* write to this address to start temperature conversion */  //开始温度转换
#define ADDR_DATA							0x00	/* address of 3 bytes / 32bit pressure data */  //压力数据
#define ADDR_PROM_SETUP				0xA0	/* address of 8x 2 bytes factory and calibration data */   //工厂校准数据
#define ADDR_PROM_C1					0xA2	/* address of 6x 2 bytes calibration data */       //校准数据

/* interface ioctls */
#define IOCTL_RESET			2
#define IOCTL_MEASURE			3

namespace ms5611
{

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

/**
 * Grody hack for crc4()
 */
union prom_u {
	uint16_t c[8];
	prom_s s;
};
#pragma pack(pop)

extern bool crc4(uint16_t *n_prom);

} /* namespace */

/* interface factories */
extern device::Device *MS5611_spi_interface(ms5611::prom_u &prom_buf, uint8_t busnum);
extern device::Device *MS5611_i2c_interface(ms5611::prom_u &prom_buf, uint8_t busnum);
extern device::Device *MS5611_sim_interface(ms5611::prom_u &prom_buf, uint8_t busnum);
typedef device::Device *(*MS5611_constructor)(ms5611::prom_u &prom_buf, uint8_t busnum);


/**
 * @file systemlib.h
 * Definition of commonly used low-level system-call like functions.
 */

#ifndef SYSTEMLIB_H_
#define SYSTEMLIB_H_
#include <float.h>
#include <stdint.h>
#include <sched.h>

// OS specific settings defined in px4_tasks.h
#include <px4_tasks.h>

__BEGIN_DECLS

enum MULT_PORTS {
	MULT_0_US2_RXTX = 0,
	MULT_1_US2_FLOW,
	MULT_2_GPIO_12,
	MULT_COUNT
};

/* Check max multi port count */
#if (MULT_COUNT > 33)
#error "MULT_COUNT HAS TO BE LESS THAN OR EQUAL 33"
#endif

/* FMU board info, to be stored in the first 64 bytes of the FMU EEPROM */  //FMU控制板信息，在FMU EEPROM中前64个字节中
#pragma pack(push,1)
struct fmu_board_info_s {
	char header[3];				/**< {'P', 'X', '4'} */
	char board_name[20];			/**< Human readable board name, \0 terminated */
	uint8_t board_id;			/**< board ID, constantly increasing number per board */
	uint8_t board_version;			/**< board version, major * 10 + minor: v1.7 = 17 */
	uint8_t multi_port_config[MULT_COUNT];	/**< Configuration of multi ports 1-3		  */
	uint8_t reserved[33 - MULT_COUNT];	/**< Reserved space for more multi ports	  */
	uint16_t production_year;
	uint8_t production_month;
	uint8_t production_day;
	uint8_t production_fab;
	uint8_t production_tester;
}; /**< stores autopilot board information meta data from EEPROM */
#pragma pack(pop)

/* Carrier board info, to be stored in the 128 byte board info EEPROM */  //获取控制板信息
#pragma pack(push,1)
struct carrier_board_info_s {
	char header[3];				/**< {'P', 'X', '4'} */
	char board_name[20];			/**< Human readable board name, \0 terminated */
	uint8_t board_id;			/**< board ID, constantly increasing number per board */
	uint8_t board_version;			/**< board version, major * 10 + minor: v1.7 = 17 */
	uint8_t multi_port_config[MULT_COUNT];	/**< Configuration of multi ports 1-3		  */
	uint8_t reserved[33 - MULT_COUNT];	/**< Reserved space for more multi ports	  */
	uint16_t production_year;
	uint8_t production_month;
	uint8_t production_day;
	uint8_t production_fab;
	uint8_t production_tester;
	char board_custom_data[64];
}; /**< stores carrier board information meta data from EEPROM */
#pragma pack(pop)

struct __multiport_info {
	const char *port_names[MULT_COUNT];
};
__EXPORT extern const struct __multiport_info multiport_info;

__END_DECLS

#endif /* SYSTEMLIB_H_ */



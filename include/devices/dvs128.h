/**
 * @file dvs128.h
 *
 * DVS128 specific configuration defines and information structures.
 */

#ifndef LIBCAER_DEVICES_DVS128_H_
#define LIBCAER_DEVICES_DVS128_H_

#include "usb.h"
#include "../events/polarity.h"
#include "../events/special.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device type definition for iniLabs DVS128.
 */
#define CAER_DEVICE_DVS128 0

/**
 * Module address: device-side DVS configuration.
 */
#define DVS128_CONFIG_DVS  0
/**
 * Module address: device-side chip bias generator configuration.
 */
#define DVS128_CONFIG_BIAS 1

/**
 * Parameter address for module DVS128_CONFIG_DVS:
 * run the DVS chip and generate polarity event data.
 */
#define DVS128_CONFIG_DVS_RUN             0
/**
 * Parameter address for module DVS128_CONFIG_DVS:
 * reset the time-stamp counter of the device. This is a temporary
 * configuration switch and will reset itself right away.
 */
#define DVS128_CONFIG_DVS_TIMESTAMP_RESET 1
/**
 * Parameter address for module DVS128_CONFIG_DVS:
 * reset the whole DVS pixel array. This is a temporary
 * configuration switch and will reset itself right away.
 */
#define DVS128_CONFIG_DVS_ARRAY_RESET     2
/**
 * Parameter address for module DVS128_CONFIG_DVS:
 * control if this DVS is a timestamp master device.
 * Default is enabled.
 */
#define DVS128_CONFIG_DVS_TS_MASTER       3

/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * First stage amplifier cascode bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_CAS     0
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Injected ground bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_INJGND  1
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Pull down on chip request (AER).
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_REQPD   2
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Pull up on request from X arbiter (AER).
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_PUX     3
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Off events threshold bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_DIFFOFF 4
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Pull down for passive load inverters in digital AER pixel circuitry.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_REQ     5
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Refractory period bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_REFR    6
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Pull up on request from Y arbiter (AER).
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_PUY     7
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * On events threshold bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_DIFFON  8
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Differential (second stage amplifier) bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_DIFF    9
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Source follower bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_FOLL    10
/**
 * Parameter address for module DVS128_CONFIG_BIAS:
 * Photoreceptor bias.
 * See 'http://inilabs.com/support/biasing/' for more details.
 */
#define DVS128_CONFIG_BIAS_PR      11

/**
 * DVS128 device-related information.
 */
struct caer_dvs128_info {
	/// Unique device identifier. Also 'source' for events.
	int16_t deviceID;
	/// Device serial number.
	char deviceSerialNumber[8 + 1];
	/// Device USB bus number.
	uint8_t deviceUSBBusNumber;
	/// Device USB device address.
	uint8_t deviceUSBDeviceAddress;
	/// Device information string, for logging purposes.
	char *deviceString;
	/// Logic (FPGA/CPLD) version.
	int16_t logicVersion;
	/// Whether the device is a time-stamp master or slave.
	bool deviceIsMaster;
	/// DVS X axis resolution.
	int16_t dvsSizeX;
	/// DVS Y axis resolution.
	int16_t dvsSizeY;
};

/**
* Info handle used for device discovery
*/
struct dvs128_info_handle {
	uint16_t deviceType;
	struct caer_dvs128_info info;
};

typedef struct dvs128_info_handle *dvs128InfoHandle;

/**
 * Return basic information on the device, such as its ID, its
 * resolution, the logic version, and so on. See the 'struct
 * caer_dvs128_info' documentation for more details.
 *
 * @param handle a valid device handle.
 *
 * @return a copy of the device information structure if successful,
 *         an empty structure (all zeros) on failure.
 */
struct caer_dvs128_info caerDVS128InfoGet(caerDeviceHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_DVS128_H_ */

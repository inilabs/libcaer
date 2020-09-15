#ifndef LIBCAER_SRC_DVXPLORER_H_
#define LIBCAER_SRC_DVXPLORER_H_

#include "libcaer/devices/device_discover.h"
#include "libcaer/devices/dvxplorer.h"

#include "container_generation.h"
#include "data_exchange.h"
#include "usb_utils.h"

#define IMU_TYPE_TEMP   0x01
#define IMU_TYPE_GYRO   0x02
#define IMU_TYPE_ACCEL  0x04
#define IMU_TOTAL_COUNT 14

#define IMU6_EVENT_PKT_POS    2
#define DVXPLORER_EVENT_TYPES 3

#define DVXPLORER_POLARITY_DEFAULT_SIZE 8192
#define DVXPLORER_SPECIAL_DEFAULT_SIZE  128
#define DVXPLORER_IMU_DEFAULT_SIZE      64

#define DVXPLORER_DEVICE_NAME "DVXplorer"

#define DVXPLORER_DEVICE_PID                 0x8419
#define DVXPLORER_REQUIRED_LOGIC_VERSION     18
#define DVXPLORER_REQUIRED_LOGIC_PATCH_LEVEL 2
#define DVXPLORER_REQUIRED_FIRMWARE_VERSION  8

#define DEBUG_ENDPOINT      0x81
#define DEBUG_TRANSFER_NUM  4
#define DEBUG_TRANSFER_SIZE 64

#define DEVICE_DVS 5

#define REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST      0x000B
#define REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR 0x000C
#define REGISTER_BIAS_OTP_TRIM                              0x000D
#define REGISTER_BIAS_PINS_DBGP                             0x000F
#define REGISTER_BIAS_PINS_DBGN                             0x0010
#define REGISTER_BIAS_CURRENT_LEVEL_SFOFF                   0x0012
#define REGISTER_BIAS_PINS_BUFP                             0x0013
#define REGISTER_BIAS_PINS_BUFN                             0x0014
#define REGISTER_BIAS_PINS_DOB                              0x0015
#define REGISTER_BIAS_CURRENT_AMP                           0x0018
#define REGISTER_BIAS_CURRENT_ON                            0x001C
#define REGISTER_BIAS_CURRENT_OFF                           0x001E

#define REGISTER_CONTROL_MODE                   0x3000
#define REGISTER_CONTROL_INTERRUPT_SOURCE       0x3004
#define REGISTER_CONTROL_INTERRUPT_ENABLE_TIME  0x3005
#define REGISTER_CONTROL_INTERRUPT_ACKNOWLEDGE  0x3007
#define REGISTER_CONTROL_INTERRUPT_AUTO_MODE    0x3008
#define REGISTER_CONTROL_INTERRUPT_RELEASE_TIME 0x3009

#define REGISTER_CONTROL_PLL_P                0x300D
#define REGISTER_CONTROL_PLL_M                0x300E
#define REGISTER_CONTROL_PLL_S                0x3010
#define REGISTER_CONTROL_CLOCK_DIVIDER_SYS    0x3011
#define REGISTER_CONTROL_CLOCK_DIVIDER_PVI    0x3012
#define REGISTER_CONTROL_PARALLEL_OUT_CONTROL 0x3019
#define REGISTER_CONTROL_PARALLEL_OUT_ENABLE  0x301E
#define REGISTER_CONTROL_PACKET_FORMAT        0x3067

#define REGISTER_DIGITAL_ENABLE               0x3200
#define REGISTER_DIGITAL_RESTART              0x3201
#define REGISTER_DIGITAL_DUAL_BINNING         0x3202
#define REGISTER_DIGITAL_SUBSAMPLE_RATIO      0x3204
#define REGISTER_DIGITAL_AREA_BLOCK           0x3205
#define REGISTER_DIGITAL_TIMESTAMP_SUBUNIT    0x3234
#define REGISTER_DIGITAL_TIMESTAMP_RESET      0x3238
#define REGISTER_TIMING_FIRST_SELX_START      0x323C
#define REGISTER_TIMING_GH_COUNT              0x3240
#define REGISTER_TIMING_GH_COUNT_FINE         0x3243
#define REGISTER_TIMING_GRS_COUNT             0x3244
#define REGISTER_TIMING_GRS_COUNT_FINE        0x3247
#define REGISTER_DIGITAL_GLOBAL_RESET_READOUT 0x3248
#define REGISTER_TIMING_NEXT_GH_CNT           0x324B
#define REGISTER_TIMING_SELX_WIDTH            0x324C
#define REGISTER_TIMING_AY_START              0x324E
#define REGISTER_TIMING_AY_END                0x324F
#define REGISTER_TIMING_MAX_EVENT_NUM         0x3251
#define REGISTER_TIMING_R_START               0x3253
#define REGISTER_TIMING_R_END                 0x3254
#define REGISTER_DIGITAL_MODE_CONTROL         0x3255
#define REGISTER_TIMING_GRS_END               0x3256
#define REGISTER_TIMING_GRS_END_FINE          0x3259
#define REGISTER_DIGITAL_FIXED_READ_TIME      0x325C
#define REGISTER_TIMING_READ_TIME_INTERVAL    0x325D
#define REGISTER_DIGITAL_EXTERNAL_TRIGGER     0x3260
#define REGISTER_TIMING_NEXT_SELX_START       0x3261
#define REGISTER_DIGITAL_BOOT_SEQUENCE        0x3266

#define REGISTER_CROPPER_BYPASS          0x3300
#define REGISTER_CROPPER_Y_START_GROUP   0x3301
#define REGISTER_CROPPER_Y_START_MASK    0x3302
#define REGISTER_CROPPER_Y_END_GROUP     0x3303
#define REGISTER_CROPPER_Y_END_MASK      0x3304
#define REGISTER_CROPPER_X_START_ADDRESS 0x3305
#define REGISTER_CROPPER_X_END_ADDRESS   0x3307

#define REGISTER_ACTIVITY_DECISION_BYPASS        0x3500
#define REGISTER_ACTIVITY_DECISION_POS_THRESHOLD 0x3501
#define REGISTER_ACTIVITY_DECISION_NEG_THRESHOLD 0x3503
#define REGISTER_ACTIVITY_DECISION_DEC_RATE      0x3505
#define REGISTER_ACTIVITY_DECISION_DEC_TIME      0x3506
#define REGISTER_ACTIVITY_DECISION_POS_MAX_COUNT 0x3513

#define REGISTER_SPATIAL_HISTOGRAM_OFF 0x3600

struct dvxplorer_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	struct {
		// DVS specific fields
		uint16_t lastX;
		uint16_t lastYG1;
		uint16_t lastYG2;
		int16_t sizeX;
		int16_t sizeY;
		bool flipX;
		bool flipY;
		bool invertXY;
		uint16_t cropperYStart;
		uint16_t cropperYEnd;
	} dvs;
	struct {
		// IMU specific fields
		bool ignoreEvents;
		bool flipX;
		bool flipY;
		bool flipZ;
		uint8_t type;
		uint8_t count;
		uint8_t tmpData;
		float accelScale;
		float gyroScale;
		// Current composite events, for later copy, to not loose them on commits.
		struct caer_imu6_event currentEvent;
	} imu;
	// Packet Container state
	struct container_generation container;
	struct {
		// Polarity Packet state
		caerPolarityEventPacket polarity;
		int32_t polarityPosition;
		// IMU6 Packet state
		caerIMU6EventPacket imu6;
		int32_t imu6Position;
		// Special Packet state
		caerSpecialEventPacket special;
		int32_t specialPosition;
	} currentPackets;
	// Device timing data.
	struct {
		uint16_t logicClock;
		uint16_t usbClock;
		uint16_t clockDeviationFactor;
		float logicClockActual;
		float usbClockActual;
	} deviceClocks;
	struct {
		// Debug transfer support (FX3 only).
		struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
		atomic_uint_fast32_t activeDebugTransfers;
	} fx3Support;
};

typedef struct dvxplorer_state *dvXplorerState;

struct dvxplorer_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_dvx_info info;
	// State for data management, common to all DVXPLORER.
	struct dvxplorer_state state;
};

typedef struct dvxplorer_handle *dvXplorerHandle;

ssize_t dvXplorerFind(caerDeviceDiscoveryResult *discoveredDevices);

caerDeviceHandle dvXplorerOpen(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
bool dvXplorerClose(caerDeviceHandle cdh);

bool dvXplorerSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool dvXplorerConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool dvXplorerConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool dvXplorerDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool dvXplorerDataStop(caerDeviceHandle handle);
caerEventPacketContainer dvXplorerDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DVXPLORER_H_ */

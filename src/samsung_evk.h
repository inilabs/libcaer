#ifndef LIBCAER_SRC_SAMSUNG_EVK_H_
#define LIBCAER_SRC_SAMSUNG_EVK_H_

#include "libcaer/devices/device_discover.h"
#include "libcaer/devices/samsung_evk.h"

#include "container_generation.h"
#include "data_exchange.h"
#include "usb_utils.h"

#define SAMSUNG_EVK_EVENT_TYPES 2

#define SAMSUNG_EVK_POLARITY_DEFAULT_SIZE 8192
#define SAMSUNG_EVK_SPECIAL_DEFAULT_SIZE  128

#define SAMSUNG_EVK_DEVICE_NAME "Samsung EVK"

#define SAMSUNG_EVK_DEVICE_VID 0x04B4
#define SAMSUNG_EVK_DEVICE_PID 0x00F1

#define SAMSUNG_EVK_DATA_ENDPOINT 0x81

#define VENDOR_REQUEST_I2C_WRITE 0xBA
#define VENDOR_REQUEST_I2C_READ  0xBB
#define VENDOR_REQUEST_RESET     0xBC

#define DEVICE_FPGA 0x0040
#define DEVICE_DVS  0x0020

#define REGISTER_RC0S_BIAS_OTP_TRIM  0x0166
#define REGISTER_RC0S_BIAS_CTRL_CRGS 0x0157
#define REGISTER_RC0S_BIAS_ON        0x0167
#define REGISTER_RC0S_BIAS_OFF       0x0168

#define REGISTER_RC0S_CONTROL_MODE                0x3000
#define REGISTER_RC0S_UNK_001                     0x300B
#define REGISTER_RC0S_UNK_002                     0x3016
#define REGISTER_RC0S_UNK_003                     0x3032
#define REGISTER_RC0S_UNK_004                     0x3033
#define REGISTER_RC0S_UNK_005                     0x3034
#define REGISTER_RC0S_UNK_006                     0x3040
#define REGISTER_RC0S_UNK_007                     0x3048
#define REGISTER_RC0S_CONTROL_PLL_P               0x3070
#define REGISTER_RC0S_CONTROL_PLL_M               0x3071
#define REGISTER_RC0S_CONTROL_PLL_S               0x3073
#define REGISTER_RC0S_UNK_008                     0x3079
#define REGISTER_RC0S_CONTROL_CLOCK_DIVIDER_VS_VP 0x307B
#define REGISTER_RC0S_UNK_009                     0x307E
#define REGISTER_RC0S_CONTROL_SBLOCK_CLK_EN1      0x3080
#define REGISTER_RC0S_CONTROL_SBLOCK_CLK_EN2      0x3081
#define REGISTER_RC0S_UNK_010                     0x3082
#define REGISTER_RC0S_UNK_011                     0x3083
#define REGISTER_RC0S_UNK_012                     0x30A0
#define REGISTER_RC0S_UNK_013                     0x30A1
#define REGISTER_RC0S_UNK_014                     0x30A2
#define REGISTER_RC0S_UNK_015                     0x30A3

#define REGISTER_RC0S_CONTROL_OUTPUT_MUX 0x311F
#define REGISTER_RC0S_UNK_016            0x3130
#define REGISTER_RC0S_UNK_017            0x3131
#define REGISTER_RC0S_UNK_018            0x3178
#define REGISTER_RC0S_UNK_019            0x3179
#define REGISTER_RC0S_UNK_020            0x317A
#define REGISTER_RC0S_UNK_021            0x317B
#define REGISTER_RC0S_UNK_022            0x317D
#define REGISTER_RC0S_UNK_023            0x317E
#define REGISTER_RC0S_UNK_024            0x31F0

#define REGISTER_RC0S_UNK_025              0x3201
#define REGISTER_RC0S_UNK_026              0x322D
#define REGISTER_RC0S_UNK_027              0x3240
#define REGISTER_RC0S_UNK_028              0x3241
#define REGISTER_RC0S_UNK_029              0x3242
#define REGISTER_RC0S_UNK_030              0x3243
#define REGISTER_RC0S_UNK_031              0x3244
#define REGISTER_RC0S_UNK_032              0x3245
#define REGISTER_RC0S_UNK_033              0x3246
#define REGISTER_RC0S_UNK_034              0x3247
#define REGISTER_RC0S_UNK_035              0x3248
#define REGISTER_RC0S_UNK_036              0x3249
#define REGISTER_RC0S_UNK_037              0x324A
#define REGISTER_RC0S_DIGITAL_MODE_CONTROL 0x3255
#define REGISTER_RC0S_UNK_038              0x3256
#define REGISTER_RC0S_UNK_039              0x3257
#define REGISTER_RC0S_UNK_040              0x3258
#define REGISTER_RC0S_UNK_041              0x3259
#define REGISTER_RC0S_UNK_042              0x325A
#define REGISTER_RC0S_UNK_043              0x325B
#define REGISTER_RC0S_UNK_044              0x325F
#define REGISTER_RC0S_UNK_045              0x3260
#define REGISTER_RC0S_TIMING_GH_SET        0x32C0
#define REGISTER_RC0S_TIMING_GL_SET        0x32C1
#define REGISTER_RC0S_TIMING_GR            0x32C2
#define REGISTER_RC0S_TIMING_GL_HOLD       0x32C3
#define REGISTER_RC0S_TIMING_SELX          0x32C4
#define REGISTER_RC0S_TIMING_SENSE         0x32C5
#define REGISTER_RC0S_TIMING_AY            0x32C6
#define REGISTER_RC0S_TIMING_AY_RST_GAP    0x32C7
#define REGISTER_RC0S_TIMING_APS_RST       0x32C8
#define REGISTER_RC0S_TIMING_SELX_GAP      0x32C9
#define REGISTER_RC0S_TIMING_COL_MARGIN    0x32CA
#define REGISTER_RC0S_TIMING_FRM_MARGIN    0x32CB
#define REGISTER_RC0S_UNK_046              0x32E1
#define REGISTER_RC0S_DIGITAL_SUBSAMPLE    0x32E3

#define REGISTER_RC0S_CROPPER_BYPASS          0x3300
#define REGISTER_RC0S_CROPPER_Y_START_GROUP   0x3301
#define REGISTER_RC0S_CROPPER_Y_START_MASK    0x3302
#define REGISTER_RC0S_CROPPER_Y_END_GROUP     0x3303
#define REGISTER_RC0S_CROPPER_Y_END_MASK      0x3304
#define REGISTER_RC0S_CROPPER_X_START_ADDRESS 0x3305
#define REGISTER_RC0S_CROPPER_X_END_ADDRESS   0x3307
#define REGISTER_RC0S_UNK_047                 0x3309
#define REGISTER_RC0S_UNK_048                 0x330A
#define REGISTER_RC0S_UNK_049                 0x330B

#define REGISTER_RC0S_ACTIVITY_DECISION_BYPASS        0x3500
#define REGISTER_RC0S_ACTIVITY_DECISION_POS_THRESHOLD 0x3501
#define REGISTER_RC0S_ACTIVITY_DECISION_NEG_THRESHOLD 0x3504
#define REGISTER_RC0S_ACTIVITY_DECISION_DEC_RATE      0x3507
#define REGISTER_RC0S_ACTIVITY_DECISION_TIME_OUT      0x3508
#define REGISTER_RC0S_ACTIVITY_DECISION_UNIT_COUNT    0x3509
#define REGISTER_RC0S_UNK_050                         0x3510
#define REGISTER_RC0S_ACTIVITY_DECISION_POS_MAX_COUNT 0x3517

#define REGISTER_RC0S_SPATIAL_HISTOGRAM_OFF 0x3600
#define REGISTER_RC0S_UNK_051               0x3601
#define REGISTER_RC0S_UNK_052               0x3602
#define REGISTER_RC0S_UNK_053               0x3603
#define REGISTER_RC0S_UNK_054               0x3604
#define REGISTER_RC0S_UNK_055               0x3612
#define REGISTER_RC0S_UNK_056               0x3F4F

#define REGISTER_RC0S_UNK_057 0x4008
#define REGISTER_RC0S_UNK_058 0x4014
#define REGISTER_RC0S_UNK_059 0x4300
#define REGISTER_RC0S_UNK_060 0x4308
#define REGISTER_RC0S_UNK_061 0x491C
#define REGISTER_RC0S_UNK_062 0x4D04
#define REGISTER_RC0S_UNK_063 0x4D08

#define REGISTER_231Y_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST      0x000B
#define REGISTER_231Y_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR 0x000C
#define REGISTER_231Y_BIAS_OTP_TRIM                              0x000D
#define REGISTER_231Y_BIAS_PINS_DBGP                             0x000F
#define REGISTER_231Y_BIAS_PINS_DBGN                             0x0010
#define REGISTER_231Y_BIAS_CURRENT_LEVEL_SFOFF                   0x0012
#define REGISTER_231Y_BIAS_PINS_BUFP                             0x0013
#define REGISTER_231Y_BIAS_PINS_BUFN                             0x0014
#define REGISTER_231Y_BIAS_PINS_DOB                              0x0015
#define REGISTER_231Y_BIAS_CURRENT_AMP                           0x0018
#define REGISTER_231Y_BIAS_CURRENT_ON                            0x001C
#define REGISTER_231Y_BIAS_CURRENT_OFF                           0x001E

#define REGISTER_231Y_CONTROL_MODE                   0x3000
#define REGISTER_231Y_CONTROL_INTERRUPT_SOURCE       0x3004
#define REGISTER_231Y_CONTROL_INTERRUPT_ENABLE_TIME  0x3005
#define REGISTER_231Y_CONTROL_INTERRUPT_ACKNOWLEDGE  0x3007
#define REGISTER_231Y_CONTROL_INTERRUPT_AUTO_MODE    0x3008
#define REGISTER_231Y_CONTROL_INTERRUPT_RELEASE_TIME 0x3009

#define REGISTER_231Y_CONTROL_PLL_P                0x300D
#define REGISTER_231Y_CONTROL_PLL_M                0x300E
#define REGISTER_231Y_CONTROL_PLL_S                0x3010
#define REGISTER_231Y_CONTROL_CLOCK_DIVIDER_SYS    0x3011
#define REGISTER_231Y_CONTROL_CLOCK_DIVIDER_PVI    0x3012
#define REGISTER_231Y_CONTROL_PARALLEL_OUT_CONTROL 0x3019
#define REGISTER_231Y_CONTROL_PARALLEL_OUT_ENABLE  0x301E
#define REGISTER_231Y_CONTROL_PACKET_FORMAT        0x3067

#define REGISTER_231Y_DIGITAL_ENABLE               0x3200
#define REGISTER_231Y_DIGITAL_RESTART              0x3201
#define REGISTER_231Y_DIGITAL_DUAL_BINNING         0x3202
#define REGISTER_231Y_DIGITAL_SUBSAMPLE_RATIO      0x3204
#define REGISTER_231Y_DIGITAL_AREA_BLOCK           0x3205
#define REGISTER_231Y_DIGITAL_TIMESTAMP_SUBUNIT    0x3234
#define REGISTER_231Y_DIGITAL_TIMESTAMP_RESET      0x3238
#define REGISTER_231Y_TIMING_FIRST_SELX_START      0x323C
#define REGISTER_231Y_TIMING_GH_COUNT              0x3240
#define REGISTER_231Y_TIMING_GH_COUNT_FINE         0x3243
#define REGISTER_231Y_TIMING_GRS_COUNT             0x3244
#define REGISTER_231Y_TIMING_GRS_COUNT_FINE        0x3247
#define REGISTER_231Y_DIGITAL_GLOBAL_RESET_READOUT 0x3248
#define REGISTER_231Y_TIMING_NEXT_GH_CNT           0x324B
#define REGISTER_231Y_TIMING_SELX_WIDTH            0x324C
#define REGISTER_231Y_TIMING_AY_START              0x324E
#define REGISTER_231Y_TIMING_AY_END                0x324F
#define REGISTER_231Y_TIMING_MAX_EVENT_NUM         0x3251
#define REGISTER_231Y_TIMING_R_START               0x3253
#define REGISTER_231Y_TIMING_R_END                 0x3254
#define REGISTER_231Y_DIGITAL_MODE_CONTROL         0x3255
#define REGISTER_231Y_TIMING_GRS_END               0x3256
#define REGISTER_231Y_TIMING_GRS_END_FINE          0x3259
#define REGISTER_231Y_DIGITAL_FIXED_READ_TIME      0x325C
#define REGISTER_231Y_TIMING_READ_TIME_INTERVAL    0x325D
#define REGISTER_231Y_DIGITAL_EXTERNAL_TRIGGER     0x3260
#define REGISTER_231Y_TIMING_NEXT_SELX_START       0x3261
#define REGISTER_231Y_DIGITAL_BOOT_SEQUENCE        0x3266

#define REGISTER_231Y_CROPPER_BYPASS          0x3300
#define REGISTER_231Y_CROPPER_Y_START_GROUP   0x3301
#define REGISTER_231Y_CROPPER_Y_START_MASK    0x3302
#define REGISTER_231Y_CROPPER_Y_END_GROUP     0x3303
#define REGISTER_231Y_CROPPER_Y_END_MASK      0x3304
#define REGISTER_231Y_CROPPER_X_START_ADDRESS 0x3305
#define REGISTER_231Y_CROPPER_X_END_ADDRESS   0x3307

#define REGISTER_231Y_ACTIVITY_DECISION_BYPASS        0x3500
#define REGISTER_231Y_ACTIVITY_DECISION_POS_THRESHOLD 0x3501
#define REGISTER_231Y_ACTIVITY_DECISION_NEG_THRESHOLD 0x3503
#define REGISTER_231Y_ACTIVITY_DECISION_DEC_RATE      0x3505
#define REGISTER_231Y_ACTIVITY_DECISION_DEC_TIME      0x3506
#define REGISTER_231Y_ACTIVITY_DECISION_POS_MAX_COUNT 0x3513

#define REGISTER_231Y_SPATIAL_HISTOGRAM_OFF 0x3600

struct samsung_evk_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	struct {
		// evk timestamping.
		uint64_t reference;
		uint64_t referenceOverflow;
		uint32_t lastReference;
		uint64_t lastUsedReference;
		uint32_t lastUsedSub;
		// libcaer timestamping.
		int32_t last;
		int32_t current;
		int32_t wrapOverflow;
	} timestamps;
	struct {
		// DVS specific fields
		uint16_t lastX;
		uint16_t cropperYStart;
		uint16_t cropperYEnd;
	} dvs;
	// Packet Container state
	struct container_generation container;
	struct {
		// Polarity Packet state
		caerPolarityEventPacket polarity;
		int32_t polarityPosition;
		// Special Packet state
		caerSpecialEventPacket special;
		int32_t specialPosition;
	} currentPackets;
};

typedef struct samsung_evk_state *samsungEVKState;

struct samsung_evk_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_samsung_evk_info info;
	// State for data management, common to all SAMSUNG_EVK.
	struct samsung_evk_state state;
};

typedef struct samsung_evk_handle *samsungEVKHandle;

ssize_t samsungEVKFind(caerDeviceDiscoveryResult *discoveredDevices);

caerDeviceHandle samsungEVKOpen(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
bool samsungEVKClose(caerDeviceHandle cdh);

bool samsungEVKSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool samsungEVKConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool samsungEVKConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool samsungEVKDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool samsungEVKDataStop(caerDeviceHandle handle);
caerEventPacketContainer samsungEVKDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_SAMSUNG_EVK_H_ */

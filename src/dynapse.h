#ifndef LIBCAER_SRC_DYNAPSE_H_
#define LIBCAER_SRC_DYNAPSE_H_

#include "devices/dynapse.h"
#include "data_exchange.h"
#include "container_generation.h"
#include "usb_utils.h"

#define DYNAPSE_DEVICE_NAME "Dynap-se"

#define DYNAPSE_DEVICE_PID 0x841D

#define DYNAPSE_REQUIRED_LOGIC_REVISION 5
#define DYNAPSE_REQUIRED_FIRMWARE_VERSION 3

#define VENDOR_REQUEST_FPGA_CONFIG_AER          0xC5
#define VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE 0xC6

#define DYNAPSE_EVENT_TYPES 2
#define DYNAPSE_SPIKE_EVENT_POS 1

#define DYNAPSE_SPIKE_DEFAULT_SIZE 4096
#define DYNAPSE_SPECIAL_DEFAULT_SIZE 128

#define SPI_CONFIG_MSG_SIZE 6
#define SPI_CONFIG_MAX      85

#define DYNAPSE_FX2_USB_CLOCK_FREQ 30

// Chip ID 0 cannot be used for USB output, so we have to shift it by
// adding 1, and also adding that to all others.
#define DYNAPSE_CHIPID_SHIFT 1

struct dynapse_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	// Packet Container state
	struct container_generation container;
	struct {
		// Spike Packet state
		caerSpikeEventPacket spike;
		int32_t spikePosition;
		// Special Packet state
		caerSpecialEventPacket special;
		int32_t specialPosition;
	} currentPackets;
};

typedef struct dynapse_state *dynapseState;

struct dynapse_handle {
	uint16_t deviceType;
	// Information fields.
	struct caer_dynapse_info info;
	// State for data management.
	struct dynapse_state state;
};

typedef struct dynapse_handle *dynapseHandle;

caerDeviceHandle dynapseOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
bool dynapseClose(caerDeviceHandle handle);

bool dynapseSendDefaultConfig(caerDeviceHandle handle);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool dynapseConfigSet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool dynapseConfigGet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool dynapseDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool dynapseDataStop(caerDeviceHandle handle);
caerEventPacketContainer dynapseDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DYNAPSE_H_ */

#ifndef LIBCAER_SRC_USB_UTILS_H_
#define LIBCAER_SRC_USB_UTILS_H_

#include "libcaer.h"
#include "devices/usb.h"
#include <libusb.h>
#include <stdatomic.h>

#if defined(HAVE_PTHREADS)
#include "c11threads_posix.h"
#endif

#define MAX_SERIAL_NUMBER_LENGTH 8

#define USB_DEFAULT_DEVICE_VID 0x152A

#define USB_DEFAULT_DATA_ENDPOINT 0x82

#define VENDOR_REQUEST_FPGA_CONFIG          0xBF
#define VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE 0xC2

enum { TRANS_STOPPED = 0, TRANS_RUNNING = 1 };

struct usb_state {
	// Per-device log-level (USB functions)
	atomic_uint_fast8_t usbLogLevel;
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// USB thread state
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	thrd_t usbThread;
	atomic_bool usbThreadRun;
	// USB Data Transfers
	atomic_uint_fast32_t usbBufferNumber;
	atomic_uint_fast32_t usbBufferSize;
	uint8_t dataEndPoint;
	atomic_uint_fast32_t dataTransfersRun;
	mtx_t dataTransfersLock;
	struct libusb_transfer **dataTransfers; // LOCK PROTECTED.
	uint32_t dataTransfersLength; // LOCK PROTECTED.
	atomic_uint_fast32_t activeDataTransfers;
	uint32_t failedDataTransfers;
	// USB Data Transfers handling callback
	void (*usbDataCallback)(void *usbDataCallbackPtr, const uint8_t *buffer, size_t bytesSent);
	void *usbDataCallbackPtr;
	// USB Data Transfers shutdown callback
	void (*usbShutdownCallback)(void *usbShutdownCallbackPtr);
	void *usbShutdownCallbackPtr;
};

typedef struct usb_state *usbState;

struct usb_info {
	uint8_t busNumber;
	uint8_t devAddress;
	uint16_t productID;
	char serialNumber[MAX_SERIAL_NUMBER_LENGTH + 1];
	char *deviceString;
};

bool usbDeviceOpen(usbState state, uint16_t devVID, uint16_t devPID, uint8_t busNumber, uint8_t devAddress,
	const char *serialNumber, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion);
void usbDeviceClose(usbState state);

ssize_t usbDiscoverDevices(struct usb_info ***infoBuffer);

void usbSetThreadName(usbState state, const char *threadName);
void usbSetDataCallback(usbState state,
	void (*usbDataCallback)(void *usbDataCallbackPtr, const uint8_t *buffer, size_t bytesSent), void *usbDataCallbackPtr);
void usbSetShutdownCallback(usbState state, void (*usbShutdownCallback)(void *usbShutdownCallbackPtr),
	void *usbShutdownCallbackPtr);
void usbSetDataEndpoint(usbState state, uint8_t dataEndPoint);
void usbSetTransfersNumber(usbState state, uint32_t transfersNumber);
void usbSetTransfersSize(usbState state, uint32_t transfersSize);
uint32_t usbGetTransfersNumber(usbState state);
uint32_t usbGetTransfersSize(usbState state);

static inline bool usbConfigSet(usbState state, uint8_t paramAddr, uint32_t param) {
	switch (paramAddr) {
		case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
			usbSetTransfersNumber(state, param);
			break;

		case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
			usbSetTransfersSize(state, param);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

static inline bool usbConfigGet(usbState state, uint8_t paramAddr, uint32_t *param) {
	switch (paramAddr) {
		case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
			*param = usbGetTransfersNumber(state);
			break;

		case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
			*param = usbGetTransfersSize(state);
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

struct usb_info usbGenerateInfo(usbState state, const char *deviceName, uint16_t deviceID);

bool usbThreadStart(usbState state);
void usbThreadStop(usbState state);

static inline bool usbDataTransfersAreRunning(usbState state) {
	return (atomic_load(&state->dataTransfersRun) == TRANS_RUNNING);
}
bool usbDataTransfersStart(usbState state);
void usbDataTransfersStop(usbState state);

bool usbControlTransferOutAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status), void *controlOutCallbackPtr);
bool usbControlTransferInAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, size_t dataSize,
	void (*controlInCallback)(void *controlInCallbackPtr, int status, const uint8_t *buffer, size_t bufferSize),
	void *controlInCallbackPtr);
bool usbControlTransferOut(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize);
bool usbControlTransferIn(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize);

bool spiConfigSend(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
bool spiConfigSendAsync(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param,
	void (*configSendCallback)(void *configSendCallbackPtr, int status), void *configSendCallbackPtr);
bool spiConfigReceive(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param);
bool spiConfigReceiveAsync(usbState state, uint8_t moduleAddr, uint8_t paramAddr,
	void (*configReceiveCallback)(void *configReceiveCallbackPtr, int status, uint32_t param),
	void *configReceiveCallbackPtr);

#endif /* LIBCAER_SRC_USB_UTILS_H_ */

#include "devices/device.h"

#include "dvs128.h"
#include "davis.h"
#include "davis_rpi.h"
#include "dynapse.h"

#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
#include "edvs.h"
#else
#include "devices/edvs.h"
#endif

/**
 * Number of devices supported by this library.
 * 0 - CAER_DEVICE_DVS128
 * 1 - CAER_DEVICE_DAVIS_FX2
 * 2 - CAER_DEVICE_DAVIS_FX3
 * 3 - CAER_DEVICE_DYNAPSE
 * 4 - CAER_DEVICE_DAVIS
 * 5 - CAER_DEVICE_EDVS
 * 6 - CAER_DEVICE_DAVIS_RPI
 */
#define SUPPORTED_DEVICES_NUMBER 7

// Supported devices and their functions.
static caerDeviceHandle (*usbConstructors[SUPPORTED_DEVICES_NUMBER])(uint16_t deviceID, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict) = {
		[CAER_DEVICE_DVS128] = &dvs128Open,
		[CAER_DEVICE_DAVIS_FX2] = &davisFX2Open,
		[CAER_DEVICE_DAVIS_FX3] = &davisFX3Open,
		[CAER_DEVICE_DYNAPSE] = &dynapseOpen,
		[CAER_DEVICE_DAVIS] = &davisOpen,
		[CAER_DEVICE_EDVS] = NULL,
#if defined(OS_LINUX)
		[CAER_DEVICE_DAVIS_RPI] = &davisRPiOpen,
#else
		[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static caerDeviceHandle (*serialConstructors[SUPPORTED_DEVICES_NUMBER])(uint16_t deviceID, const char *serialPortName,
	uint32_t serialBaudRate) = {
		[CAER_DEVICE_DVS128] = NULL,
		[CAER_DEVICE_DAVIS_FX2] = NULL,
		[CAER_DEVICE_DAVIS_FX3] = NULL,
		[CAER_DEVICE_DYNAPSE] = NULL,
		[CAER_DEVICE_DAVIS] = NULL,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
		[CAER_DEVICE_EDVS] = &edvsOpen,
#else
		[CAER_DEVICE_EDVS] = NULL,
#endif
		[CAER_DEVICE_DAVIS_RPI] = NULL,
};

static bool (*destructors[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle) = {
	[CAER_DEVICE_DVS128] = &dvs128Close,
	[CAER_DEVICE_DAVIS_FX2] = &davisClose,
	[CAER_DEVICE_DAVIS_FX3] = &davisClose,
	[CAER_DEVICE_DYNAPSE] = &dynapseClose,
	[CAER_DEVICE_DAVIS] = &davisClose,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
	[CAER_DEVICE_EDVS] = &edvsClose,
#else
	[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
	[CAER_DEVICE_DAVIS_RPI] = &davisRPiClose,
#else
	[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static bool (*defaultConfigSenders[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle) = {
	[CAER_DEVICE_DVS128] = &dvs128SendDefaultConfig,
	[CAER_DEVICE_DAVIS_FX2] = &davisSendDefaultConfig,
	[CAER_DEVICE_DAVIS_FX3] = &davisSendDefaultConfig,
	[CAER_DEVICE_DYNAPSE] = &dynapseSendDefaultConfig,
	[CAER_DEVICE_DAVIS] = &davisSendDefaultConfig,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
	[CAER_DEVICE_EDVS] = &edvsSendDefaultConfig,
#else
	[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
	[CAER_DEVICE_DAVIS_RPI] = &davisRPiSendDefaultConfig,
#else
	[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static bool (*configSetters[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr,
	uint32_t param) = {
		[CAER_DEVICE_DVS128] = &dvs128ConfigSet,
		[CAER_DEVICE_DAVIS_FX2] = &davisConfigSet,
		[CAER_DEVICE_DAVIS_FX3] = &davisConfigSet,
		[CAER_DEVICE_DYNAPSE] = &dynapseConfigSet,
		[CAER_DEVICE_DAVIS] = &davisConfigSet,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
		[CAER_DEVICE_EDVS] = &edvsConfigSet,
#else
		[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
		[CAER_DEVICE_DAVIS_RPI] = &davisRPiConfigSet,
#else
		[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static bool (*configGetters[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr,
	uint32_t *param) = {
		[CAER_DEVICE_DVS128] = &dvs128ConfigGet,
		[CAER_DEVICE_DAVIS_FX2] = &davisConfigGet,
		[CAER_DEVICE_DAVIS_FX3] = &davisConfigGet,
		[CAER_DEVICE_DYNAPSE] = &dynapseConfigGet,
		[CAER_DEVICE_DAVIS] = &davisConfigGet,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
		[CAER_DEVICE_EDVS] = &edvsConfigGet,
#else
		[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
		[CAER_DEVICE_DAVIS_RPI] = &davisRPiConfigGet,
#else
		[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static bool (*dataStarters[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) = {
		[CAER_DEVICE_DVS128] = &dvs128DataStart,
		[CAER_DEVICE_DAVIS_FX2] = &davisDataStart,
		[CAER_DEVICE_DAVIS_FX3] = &davisDataStart,
		[CAER_DEVICE_DYNAPSE] = &dynapseDataStart,
		[CAER_DEVICE_DAVIS] = &davisDataStart,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
		[CAER_DEVICE_EDVS] = &edvsDataStart,
#else
		[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
		[CAER_DEVICE_DAVIS_RPI] = &davisRPiDataStart,
#else
		[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static bool (*dataStoppers[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle) = {
	[CAER_DEVICE_DVS128] = &dvs128DataStop,
	[CAER_DEVICE_DAVIS_FX2] = &davisDataStop,
	[CAER_DEVICE_DAVIS_FX3] = &davisDataStop,
	[CAER_DEVICE_DYNAPSE] = &dynapseDataStop,
	[CAER_DEVICE_DAVIS] = &davisDataStop,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
	[CAER_DEVICE_EDVS] = &edvsDataStop,
#else
	[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
	[CAER_DEVICE_DAVIS_RPI] = &davisRPiDataStop,
#else
	[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

static caerEventPacketContainer (*dataGetters[SUPPORTED_DEVICES_NUMBER])(caerDeviceHandle handle) = {
	[CAER_DEVICE_DVS128] = &dvs128DataGet,
	[CAER_DEVICE_DAVIS_FX2] = &davisDataGet,
	[CAER_DEVICE_DAVIS_FX3] = &davisDataGet,
	[CAER_DEVICE_DYNAPSE] = &dynapseDataGet,
	[CAER_DEVICE_DAVIS] = &davisDataGet,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
	[CAER_DEVICE_EDVS] = &edvsDataGet,
#else
	[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
	[CAER_DEVICE_DAVIS_RPI] = &davisRPiDataGet,
#else
	[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

// Add empty InfoGet for optional devices, such as serial ones.
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 0
struct caer_edvs_info caerEDVSInfoGet(caerDeviceHandle handle) {
	(void)(handle);
	struct caer_edvs_info emptyInfo = { 0, .deviceString = NULL };
	return (emptyInfo);
}
#endif

struct caer_device_handle {
	uint16_t deviceType;
	// This is compatible with all device handle structures.
	// The first member is always 'uint16_t deviceType'.
};

caerDeviceHandle caerDeviceOpen(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict,
	uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	// Check if device type is supported.
	if (deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (NULL);
	}

	// Execute main USB constructor function.
	if (usbConstructors[deviceType] == NULL) {
		return (NULL);
	}

	return (usbConstructors[deviceType](deviceID, busNumberRestrict, devAddressRestrict, serialNumberRestrict));
}

caerDeviceHandle caerDeviceOpenSerial(uint16_t deviceID, uint16_t deviceType, const char *serialPortName,
	uint32_t serialBaudRate) {
	// Check if device type is supported.
	if (deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (NULL);
	}

	// Execute main serial constructor function.
	if (serialConstructors[deviceType] == NULL) {
		return (NULL);
	}

	return (serialConstructors[deviceType](deviceID, serialPortName, serialBaudRate));
}

bool caerDeviceClose(caerDeviceHandle *handlePtr) {
	// We want a pointer here so we can ensure the reference is set to NULL.
	// Check if either it, or the memory pointed to, are NULL and abort
	// if that's the case.
	if (handlePtr == NULL) {
		return (false);
	}

	if (*handlePtr == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if ((*handlePtr)->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (false);
	}

	// Call appropriate destructor function.
	if (destructors[(*handlePtr)->deviceType] == NULL) {
		return (false);
	}

	bool retVal = destructors[(*handlePtr)->deviceType](*handlePtr);

	// Done. Set reference to NULL if successful.
	if (retVal) {
		*handlePtr = NULL;
	}

	return (retVal);
}

bool caerDeviceSendDefaultConfig(caerDeviceHandle handle) {
	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (false);
	}

	// Call appropriate function.
	if (defaultConfigSenders[handle->deviceType] == NULL) {
		return (false);
	}

	return (defaultConfigSenders[handle->deviceType](handle));
}

bool caerDeviceConfigSet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (false);
	}

	// Call appropriate function.
	if (configSetters[handle->deviceType] == NULL) {
		return (false);
	}

	return (configSetters[handle->deviceType](handle, modAddr, paramAddr, param));
}

bool caerDeviceConfigGet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (false);
	}

	// Ensure content of param is zeroed out.
	*param = 0;

	// Call appropriate function.
	if (configGetters[handle->deviceType] == NULL) {
		return (false);
	}

	return (configGetters[handle->deviceType](handle, modAddr, paramAddr, param));
}

bool caerDeviceDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (false);
	}

	// Call appropriate function.
	if (dataStarters[handle->deviceType] == NULL) {
		return (false);
	}

	return (dataStarters[handle->deviceType](handle, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr,
		dataShutdownNotify, dataShutdownUserPtr));
}

bool caerDeviceDataStop(caerDeviceHandle handle) {
	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (false);
	}

	// Call appropriate function.
	if (dataStoppers[handle->deviceType] == NULL) {
		return (false);
	}

	return (dataStoppers[handle->deviceType](handle));
}

caerEventPacketContainer caerDeviceDataGet(caerDeviceHandle handle) {
	// Check if the pointer is valid.
	if (handle == NULL) {
		return (NULL);
	}

	// Check if device type is supported.
	if (handle->deviceType >= SUPPORTED_DEVICES_NUMBER) {
		return (NULL);
	}

	// Call appropriate function.
	if (dataGetters[handle->deviceType] == NULL) {
		return (NULL);
	}

	return (dataGetters[handle->deviceType](handle));
}

bool caerDeviceConfigGet64(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint64_t *param) {
	// Ensure param is zeroed out.
	*param = 0;

	// This is implemented by doing two normal configGet() requests for 32bit
	// numbers and then concatenating them. The given address contains the upper
	// 32 bits, the +1 address the lower 32 bits. To guard against overflow, the
	// upper 32 bits are read first, then the lower, and then again the upper ones
	// to detect if they changed. If yes, we restart getting the value, because
	// an overflow must have happened. If no, the value is good to use.
	uint32_t upperBits, lowerBits, verifyUpperBits;

retry:
	if (!caerDeviceConfigGet(handle, modAddr, paramAddr, &upperBits)) {
		return (false);
	}

	if (!caerDeviceConfigGet(handle, modAddr, U8T(paramAddr + 1), &lowerBits)) {
		return (false);
	}

	if (!caerDeviceConfigGet(handle, modAddr, paramAddr, &verifyUpperBits)) {
		return (false);
	}

	// Guard against overflow while reading.
	if (upperBits != verifyUpperBits) {
		goto retry;
	}

	// Concatenate two 32bit values to one 64bit one.
	*param = U64T(U64T(upperBits) << 32) | U64T(lowerBits);

	return (true);
}

ssize_t caerDeviceDiscover(caerDeviceInfoHandle **infoBuffer) {
	struct usb_info **usbInfo = NULL;
	ssize_t res = usbDiscoverDevices(&usbInfo);
	ssize_t numDevices = 0;

	if (res <= 0) {
		return res;
	}

	*infoBuffer = calloc((size_t)res, sizeof(**infoBuffer));

	for (size_t i = 0; i < (size_t)res; i++) {
		if (usbInfo[i]->productID == DYNAPSE_DEVICE_PID) {
			caerDeviceHandle cdh = dynapseOpen(0, usbInfo[i]->busNumber, usbInfo[i]->devAddress, NULL);
			if (cdh == NULL)
				continue;
			dynapseInfoHandle dih = calloc(1, sizeof(*dih));
			dih->deviceType = CAER_DEVICE_DYNAPSE;
			dih->info = caerDynapseInfoGet(cdh);
			dynapseClose(cdh);
			(*infoBuffer)[i] = (caerDeviceInfoHandle)dih;
			numDevices++;
		}
		else if (usbInfo[i]->productID == DAVIS_FX2_DEVICE_PID || usbInfo[i]->productID == DAVIS_FX3_DEVICE_PID) {
			caerDeviceHandle cdh = davisOpen(0, usbInfo[i]->busNumber, usbInfo[i]->devAddress, NULL);
			if (cdh == NULL)
				continue;
			davisInfoHandle dih = calloc(1, sizeof(*dih));
			dih->deviceType = CAER_DEVICE_DAVIS;
			dih->info = caerDavisInfoGet(cdh);
			davisClose(cdh);
			(*infoBuffer)[i] = (caerDeviceInfoHandle)dih;
			numDevices++;
		}
		else if (usbInfo[i]->productID == DVS_DEVICE_PID) {
			caerDeviceHandle cdh = dvs128Open(0, usbInfo[i]->busNumber, usbInfo[i]->devAddress, NULL);
			if (cdh == NULL)
				continue;
			dvs128InfoHandle dih = calloc(1, sizeof(*dih));
			dih->deviceType = CAER_DEVICE_DVS128;
			dih->info = caerDVS128InfoGet(cdh);
			dvs128Close(cdh);
			(*infoBuffer)[i] = (caerDeviceInfoHandle)dih;
			numDevices++;
		}
	}

	return numDevices;
}

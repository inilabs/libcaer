#include "dvxplorer.h"

#include <math.h>

static void dvXplorerLog(enum caer_log_level logLevel, dvXplorerHandle handle, const char *format, ...)
	ATTRIBUTE_FORMAT(3);
static void dvXplorerEventTranslator(void *vhd, const uint8_t *buffer, size_t bytesSent);
static void dvXplorerTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param);

// FX3 Debug Transfer Support
static void allocateDebugTransfers(dvXplorerHandle handle);
static void cancelAndDeallocateDebugTransfers(dvXplorerHandle handle);
static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer);
static void debugTranslator(dvXplorerHandle handle, const uint8_t *buffer, size_t bytesSent);

static void dvXplorerLog(enum caer_log_level logLevel, dvXplorerHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel,
		handle->info.deviceString, format, argumentList);
	va_end(argumentList);
}

static void populateDeviceInfo(
	caerDeviceDiscoveryResult result, struct usb_info *usbInfo, libusb_device_handle *devHandle) {
	// This is a DVXplorer.
	result->deviceType         = CAER_DEVICE_DVXPLORER;
	result->deviceErrorOpen    = usbInfo->errorOpen;
	result->deviceErrorVersion = usbInfo->errorVersion;

	struct caer_dvx_info *dvxInfoPtr = &(result->deviceInfo.dvXplorerInfo);

	// SN, BusNumber, DevAddress always defined.
	// FirmwareVersion and LogicVersion either defined or zero.
	strncpy(dvxInfoPtr->deviceSerialNumber, usbInfo->serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);
	dvxInfoPtr->deviceUSBBusNumber     = usbInfo->busNumber;
	dvxInfoPtr->deviceUSBDeviceAddress = usbInfo->devAddress;
	dvxInfoPtr->firmwareVersion        = usbInfo->firmwareVersion;
	dvxInfoPtr->logicVersion           = usbInfo->logicVersion;

	if (devHandle != NULL) {
		// Populate info variables based on data from device.
		uint32_t param32 = 0;

		startupSPIConfigReceive(devHandle, DVX_SYSINFO, DVX_SYSINFO_CHIP_IDENTIFIER, &param32);
		dvxInfoPtr->chipID = I16T(param32);
		startupSPIConfigReceive(devHandle, DVX_SYSINFO, DVX_SYSINFO_DEVICE_IS_MASTER, &param32);
		dvxInfoPtr->deviceIsMaster = param32;

		startupSPIConfigReceive(devHandle, DVX_DVS, DVX_DVS_SIZE_COLUMNS, &param32);
		int16_t dvsSizeX = I16T(param32);
		startupSPIConfigReceive(devHandle, DVX_DVS, DVX_DVS_SIZE_ROWS, &param32);
		int16_t dvsSizeY = I16T(param32);

		startupSPIConfigReceive(devHandle, DVX_DVS, DVX_DVS_ORIENTATION_INFO, &param32);
		bool dvsInvertXY = param32 & 0x04;

		if (dvsInvertXY) {
			dvxInfoPtr->dvsSizeX = dvsSizeY;
			dvxInfoPtr->dvsSizeY = dvsSizeX;
		}
		else {
			dvxInfoPtr->dvsSizeX = dvsSizeX;
			dvxInfoPtr->dvsSizeY = dvsSizeY;
		}

		startupSPIConfigReceive(devHandle, DVX_IMU, DVX_IMU_TYPE, &param32);
		dvxInfoPtr->imuType = U8T(param32);

		// Extra features:
		startupSPIConfigReceive(devHandle, DVX_MUX, DVX_MUX_HAS_STATISTICS, &param32);
		dvxInfoPtr->muxHasStatistics = param32;

		startupSPIConfigReceive(devHandle, DVX_DVS, DVX_DVS_HAS_STATISTICS, &param32);
		dvxInfoPtr->dvsHasStatistics = param32;

		startupSPIConfigReceive(devHandle, DVX_EXTINPUT, DVX_EXTINPUT_HAS_GENERATOR, &param32);
		dvxInfoPtr->extInputHasGenerator = param32;
	}

	// Always unset here.
	dvxInfoPtr->deviceID     = -1;
	dvxInfoPtr->deviceString = NULL;
}

ssize_t dvXplorerFind(caerDeviceDiscoveryResult *discoveredDevices) {
	return (usbDeviceFind(USB_DEFAULT_DEVICE_VID, DVXPLORER_DEVICE_PID, DVXPLORER_REQUIRED_LOGIC_VERSION,
		DVXPLORER_REQUIRED_LOGIC_PATCH_LEVEL, DVXPLORER_REQUIRED_FIRMWARE_VERSION, discoveredDevices,
		&populateDeviceInfo));
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
	// Accelerometer scale is:
	// 0 - +- 2 g  - 16384 LSB/g
	// 1 - +- 4 g  - 8192 LSB/g
	// 2 - +- 8 g  - 4096 LSB/g
	// 3 - +- 16 g - 2048 LSB/g
	float accelScale = 65536.0F / (float) U32T(4 * (1 << imuAccelScale));

	return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
	// Invert for ascending scale:
	uint8_t imuGyroScaleAsc = U8T(4 - imuGyroScale);

	// Gyroscope ascending scale is:
	// 0 - +- 125 °/s  - 262.4 LSB/°/s
	// 1 - +- 250 °/s  - 131.2 LSB/°/s
	// 2 - +- 500 °/s  - 65.6 LSB/°/s
	// 3 - +- 1000 °/s - 32.8 LSB/°/s
	// 4 - +- 2000 °/s - 16.4 LSB/°/s
	float gyroScale = 65536.0F / (float) U32T(250 * (1 << imuGyroScaleAsc));

	return (gyroScale);
}

static inline void freeAllDataMemory(dvXplorerState state) {
	dataExchangeDestroy(&state->dataExchange);

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentPackets.polarity != NULL) {
		free(&state->currentPackets.polarity->packetHeader);
		state->currentPackets.polarity = NULL;

		containerGenerationSetPacket(&state->container, POLARITY_EVENT, NULL);
	}

	if (state->currentPackets.special != NULL) {
		free(&state->currentPackets.special->packetHeader);
		state->currentPackets.special = NULL;

		containerGenerationSetPacket(&state->container, SPECIAL_EVENT, NULL);
	}

	if (state->currentPackets.imu6 != NULL) {
		free(&state->currentPackets.imu6->packetHeader);
		state->currentPackets.imu6 = NULL;

		containerGenerationSetPacket(&state->container, IMU6_EVENT_PKT_POS, NULL);
	}

	containerGenerationDestroy(&state->container);
}

caerDeviceHandle dvXplorerOpen(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	errno = 0;

	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DVXPLORER_DEVICE_NAME);

	dvXplorerHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		errno = CAER_ERROR_MEMORY_ALLOCATION;
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_DVXPLORER;

	dvXplorerState state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	dataExchangeSettingsInit(&state->dataExchange);

	// Packet settings (size (in events) and time interval (in µs)).
	containerGenerationSettingsInit(&state->container);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	usbSetLogLevel(&state->usbState, globalLogLevel);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s %" PRIu16, DVXPLORER_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);
	handle->info.deviceString = usbThreadName; // Temporary, until replaced by full string.

	// Try to open a DVXPLORER device on a specific USB port.
	struct caer_device_discovery_result deviceInfo;

	if (!usbDeviceOpen(&state->usbState, USB_DEFAULT_DEVICE_VID, DVXPLORER_DEVICE_PID, busNumberRestrict,
			devAddressRestrict, serialNumberRestrict, DVXPLORER_REQUIRED_LOGIC_VERSION,
			DVXPLORER_REQUIRED_LOGIC_PATCH_LEVEL, DVXPLORER_REQUIRED_FIRMWARE_VERSION, &deviceInfo,
			&populateDeviceInfo)) {
		if (errno == CAER_ERROR_OPEN_ACCESS) {
			dvXplorerLog(
				CAER_LOG_CRITICAL, handle, "Failed to open device, no matching device could be found or opened.");
		}
		else {
			dvXplorerLog(CAER_LOG_CRITICAL, handle,
				"Failed to open device, see above log message for more information (errno=%d).", errno);
		}

		free(handle);

		// errno set by usbDeviceOpen().
		return (NULL);
	}

	// At this point we can get some more precise data on the device and update
	// the logging string to reflect that and be more informative.
	char *usbInfoString = malloc(USB_INFO_STRING_SIZE);
	if (usbInfoString == NULL) {
		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to generate USB information string.");

		usbDeviceClose(&state->usbState);
		free(handle);

		errno = CAER_ERROR_MEMORY_ALLOCATION;
		return (NULL);
	}

	snprintf(usbInfoString, USB_INFO_STRING_SIZE, DVXPLORER_DEVICE_NAME " ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]",
		deviceID, deviceInfo.deviceInfo.dvXplorerInfo.deviceSerialNumber,
		deviceInfo.deviceInfo.dvXplorerInfo.deviceUSBBusNumber,
		deviceInfo.deviceInfo.dvXplorerInfo.deviceUSBDeviceAddress);

	// Setup USB.
	usbSetDataCallback(&state->usbState, &dvXplorerEventTranslator, handle);
	usbSetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);
	usbSetTransfersNumber(&state->usbState, 8);
	usbSetTransfersSize(&state->usbState, 8192);

	// Start USB handling thread.
	if (!usbThreadStart(&state->usbState)) {
		usbDeviceClose(&state->usbState);
		free(usbInfoString);
		free(handle);

		errno = CAER_ERROR_COMMUNICATION;
		return (NULL);
	}

	// Populate info variables based on data from device.
	handle->info = deviceInfo.deviceInfo.dvXplorerInfo;

	handle->info.deviceID     = I16T(deviceID);
	handle->info.deviceString = usbInfoString;

	uint32_t param32 = 0;

	spiConfigReceive(&state->usbState, DVX_SYSINFO, DVX_SYSINFO_LOGIC_CLOCK, &param32);
	state->deviceClocks.logicClock = U16T(param32);
	spiConfigReceive(&state->usbState, DVX_SYSINFO, DVX_SYSINFO_USB_CLOCK, &param32);
	state->deviceClocks.usbClock = U16T(param32);
	spiConfigReceive(&state->usbState, DVX_SYSINFO, DVX_SYSINFO_CLOCK_DEVIATION, &param32);
	state->deviceClocks.clockDeviationFactor = U16T(param32);

	// Calculate actual clock frequencies.
	state->deviceClocks.logicClockActual = (float) ((double) state->deviceClocks.logicClock
													* ((double) state->deviceClocks.clockDeviationFactor / 1000.0));
	state->deviceClocks.usbClockActual   = (float) ((double) state->deviceClocks.usbClock
                                                  * ((double) state->deviceClocks.clockDeviationFactor / 1000.0));

	dvXplorerLog(CAER_LOG_DEBUG, handle, "Clock frequencies: LOGIC %f, USB %f.",
		(double) state->deviceClocks.logicClockActual, (double) state->deviceClocks.usbClockActual);

	spiConfigReceive(&state->usbState, DVX_DVS, DVX_DVS_SIZE_COLUMNS, &param32);
	state->dvs.sizeX = I16T(param32);
	spiConfigReceive(&state->usbState, DVX_DVS, DVX_DVS_SIZE_ROWS, &param32);
	state->dvs.sizeY = I16T(param32);

	spiConfigReceive(&state->usbState, DVX_DVS, DVX_DVS_ORIENTATION_INFO, &param32);
	state->dvs.invertXY = param32 & 0x04;
	state->dvs.flipX    = param32 & 0x02;
	state->dvs.flipY    = param32 & 0x01;

	dvXplorerLog(CAER_LOG_DEBUG, handle, "DVS Size X: %d, Size Y: %d, Invert: %d.", state->dvs.sizeX, state->dvs.sizeY,
		state->dvs.invertXY);

	spiConfigReceive(&state->usbState, DVX_IMU, DVX_IMU_ORIENTATION_INFO, &param32);
	state->imu.flipX = param32 & 0x04;
	state->imu.flipY = param32 & 0x02;
	state->imu.flipZ = param32 & 0x01;

	dvXplorerLog(CAER_LOG_DEBUG, handle, "IMU Flip X: %d, Flip Y: %d, Flip Z: %d.", state->imu.flipX, state->imu.flipY,
		state->imu.flipZ);

	// Initialize Samsung DVS chip.
	spiConfigSend(&state->usbState, DVX_MUX, DVX_MUX_RUN_CHIP, true); // Take DVS out of reset.

	// Wait 10ms for DVS to start.
	struct timespec dvsSleep = {.tv_sec = 0, .tv_nsec = 10000000};
	thrd_sleep(&dvsSleep, NULL);

	// Bias reset.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_OTP_TRIM, 0x24);

	// Bias enable.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_PINS_DBGP, 0x07);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_PINS_DBGN, 0xFF);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_PINS_BUFP, 0x03);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_PINS_BUFN, 0x7F);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_PINS_DOB, 0x00);

	dvXplorerConfigSet(
		(caerDeviceHandle) handle, DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, DVX_DVS_CHIP_BIAS_SIMPLE_DEFAULT);

	// System settings.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_CLOCK_DIVIDER_SYS, 0xA0); // Divide freq by 10.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PARALLEL_OUT_CONTROL, 0x00);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PARALLEL_OUT_ENABLE, 0x01);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, 0x80); // Enable MGROUP compression.

	// Digital settings.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_TIMESTAMP_SUBUNIT, 0x31);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL, 0x0C); // R/AY signals enable.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_BOOT_SEQUENCE, 0x08);

	// Fine clock counts based on clock frequency.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT_FINE, 50);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT_FINE, 50);
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END_FINE, 50);

	// Disable histogram, not currently used/mapped.
	spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_SPATIAL_HISTOGRAM_OFF, 0x01);

	// On FX3, start the debug transfers once everything else is ready.
	allocateDebugTransfers(handle);

	dvXplorerLog(CAER_LOG_DEBUG, handle, "Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		handle->info.deviceUSBBusNumber, handle->info.deviceUSBDeviceAddress);

	return ((caerDeviceHandle) handle);
}

bool dvXplorerClose(caerDeviceHandle cdh) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;
	dvXplorerState state   = &handle->state;

	dvXplorerLog(CAER_LOG_DEBUG, handle, "Shutting down ...");

	spiConfigSend(&state->usbState, DVX_MUX, DVX_MUX_RUN_CHIP, false); // Put DVS in reset.

	// Stop debug transfers on FX3 devices.
	cancelAndDeallocateDebugTransfers(handle);

	// Shut down USB handling thread.
	usbThreadStop(&state->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&state->usbState);

	dvXplorerLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	return (true);
}

struct caer_dvx_info caerDVXplorerInfoGet(caerDeviceHandle cdh) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_dvx_info emptyInfo = {0, .deviceString = NULL};
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DVXPLORER) {
		struct caer_dvx_info emptyInfo = {0, .deviceString = NULL};
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool dvXplorerSendDefaultConfig(caerDeviceHandle cdh) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;

	dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_TIMESTAMP_RESET, false);
	dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
	dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_DROP_DVS_ON_TRANSFER_STALL, false);

	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_ACCEL_DATA_RATE, BOSCH_ACCEL_800HZ); // 800 Hz.
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_ACCEL_FILTER, BOSCH_ACCEL_NORMAL);   // Normal mode.
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_ACCEL_RANGE, BOSCH_ACCEL_4G);        // +- 4 g.
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_GYRO_DATA_RATE, BOSCH_GYRO_800HZ);   // 800 Hz.
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_GYRO_FILTER, BOSCH_GYRO_NORMAL);     // Normal mode.
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_GYRO_RANGE, BOSCH_GYRO_500DPS);      // +- 500 °/s

	dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_DETECT_RISING_EDGES, false);
	dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_DETECT_FALLING_EDGES, false);
	dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_DETECT_PULSES, true);
	dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_DETECT_PULSE_POLARITY, true);
	dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_DETECT_PULSE_LENGTH,
		10); // in µs, converted to cycles @ LogicClock later

	if (handle->info.extInputHasGenerator) {
		// Disable generator by default. Has to be enabled manually after sendDefaultConfig() by user!
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_RUN_GENERATOR, false);
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_GENERATE_PULSE_POLARITY, true);
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_GENERATE_PULSE_INTERVAL,
			10); // in µs, converted to cycles @ LogicClock later
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_GENERATE_PULSE_LENGTH,
			5); // in µs, converted to cycles @ LogicClock later
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);
	}

	dvXplorerConfigSet(cdh, DVX_USB, DVX_USB_EARLY_PACKET_DELAY,
		8); // in 125µs time-slices (defaults to 1ms)

	// Set default biases.
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, DVX_DVS_CHIP_BIAS_SIMPLE_DEFAULT);

	// External trigger.
	dvXplorerConfigSet(
		cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_EXTERNAL_TRIGGER_MODE, DVX_DVS_CHIP_EXTERNAL_TRIGGER_MODE_TIMESTAMP_RESET);

	// Digital readout configuration.
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_HOLD_ENABLE, true);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_RESET_ENABLE, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_RESET_DURING_READOUT, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_FIXED_READ_TIME_ENABLE, false);

	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_ENABLE, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_DUAL_BINNING_ENABLE, false);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, DVX_DVS_CHIP_SUBSAMPLE_VERTICAL_NONE);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL_NONE);

	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_0, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_1, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_2, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_3, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_4, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_5, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_6, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_7, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_8, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_9, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_10, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_11, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_12, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_13, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_14, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_15, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_16, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_17, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_18, 0x7FFF);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_AREA_BLOCKING_19, 0x7FFF);

	// Timing settings.
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_ED, 2);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_GH2GRS, 0);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_GRS, 1);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_GH2SEL, 4);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_SELW, 6);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_SEL2AY_R, 4);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_SEL2AY_F, 6);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_SEL2R_R, 8);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_SEL2R_F, 10);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_SEL, 15);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_NEXT_GH, 10);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_TIMING_READ_FIXED, 48000);

	// Crop block.
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_ENABLE, false);

	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, 0);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, 0);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, U32T(handle->info.dvsSizeX - 1));
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, U32T(handle->info.dvsSizeY - 1));

	// Activity decision block.
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_ACTIVITY_DECISION, DVX_DVS_CHIP_ACTIVITY_DECISION_ENABLE, false);

	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_ACTIVITY_DECISION, DVX_DVS_CHIP_ACTIVITY_DECISION_POS_THRESHOLD, 300);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_ACTIVITY_DECISION, DVX_DVS_CHIP_ACTIVITY_DECISION_NEG_THRESHOLD, 20);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_ACTIVITY_DECISION, DVX_DVS_CHIP_ACTIVITY_DECISION_DEC_RATE, 1);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_ACTIVITY_DECISION, DVX_DVS_CHIP_ACTIVITY_DECISION_DEC_TIME, 3);
	dvXplorerConfigSet(cdh, DVX_DVS_CHIP_ACTIVITY_DECISION, DVX_DVS_CHIP_ACTIVITY_DECISION_POS_MAX_COUNT, 300);

	// DTAG restart after config.
	spiConfigSend(&handle->state.usbState, DEVICE_DVS, REGISTER_DIGITAL_RESTART, 0x02);

	return (true);
}

bool dvXplorerConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;
	dvXplorerState state   = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			return (usbConfigSet(&state->usbState, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigSet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigSet(&state->container, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					atomic_store(&state->deviceLogLevel, U8T(param));

					// Set USB log-level to this value too.
					usbSetLogLevel(&state->usbState, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_MUX:
			switch (paramAddr) {
				case DVX_MUX_RUN:
				case DVX_MUX_TIMESTAMP_RUN:
				case DVX_MUX_RUN_CHIP:
				case DVX_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DVX_MUX_DROP_DVS_ON_TRANSFER_STALL:
					return (spiConfigSend(&state->usbState, DVX_MUX, paramAddr, param));
					break;

				case DVX_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						struct spi_config_params spiMultiConfig[2];

						spiMultiConfig[0].moduleAddr = DVX_MUX;
						spiMultiConfig[0].paramAddr  = DVX_MUX_TIMESTAMP_RESET;
						spiMultiConfig[0].param      = true;

						spiMultiConfig[1].moduleAddr = DVX_MUX;
						spiMultiConfig[1].paramAddr  = DVX_MUX_TIMESTAMP_RESET;
						spiMultiConfig[1].param      = false;

						return (spiConfigSendMultiple(&state->usbState, spiMultiConfig, 2));
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS:
			switch (paramAddr) {
				case DVX_DVS_RUN:
					return (spiConfigSend(&state->usbState, DVX_DVS, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_IMU:
			switch (paramAddr) {
				case DVX_IMU_RUN_ACCELEROMETER:
				case DVX_IMU_RUN_GYROSCOPE:
				case DVX_IMU_RUN_TEMPERATURE:
				case DVX_IMU_ACCEL_DATA_RATE:
				case DVX_IMU_ACCEL_FILTER:
				case DVX_IMU_ACCEL_RANGE:
				case DVX_IMU_GYRO_DATA_RATE:
				case DVX_IMU_GYRO_FILTER:
				case DVX_IMU_GYRO_RANGE:
					return (spiConfigSend(&state->usbState, DVX_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_EXTINPUT:
			switch (paramAddr) {
				case DVX_EXTINPUT_RUN_DETECTOR:
				case DVX_EXTINPUT_DETECT_RISING_EDGES:
				case DVX_EXTINPUT_DETECT_FALLING_EDGES:
				case DVX_EXTINPUT_DETECT_PULSES:
				case DVX_EXTINPUT_DETECT_PULSE_POLARITY:
					return (spiConfigSend(&state->usbState, DVX_EXTINPUT, paramAddr, param));
					break;

				case DVX_EXTINPUT_DETECT_PULSE_LENGTH: {
					// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must multiply here.
					float timeCC = roundf((float) param * state->deviceClocks.logicClockActual);
					return (spiConfigSend(&state->usbState, DVX_EXTINPUT, paramAddr, U32T(timeCC)));
					break;
				}

				case DVX_EXTINPUT_RUN_GENERATOR:
				case DVX_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DVX_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DVX_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigSend(&state->usbState, DVX_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DVX_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DVX_EXTINPUT_GENERATE_PULSE_LENGTH: {
					if (handle->info.extInputHasGenerator) {
						// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
						// on FPGA, so we must multiply here.
						float timeCC = roundf((float) param * state->deviceClocks.logicClockActual);
						return (spiConfigSend(&state->usbState, DVX_EXTINPUT, paramAddr, U32T(timeCC)));
					}
					else {
						return (false);
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_SYSINFO:
			// No SystemInfo parameters can ever be set!
			return (false);
			break;

		case DVX_USB:
			switch (paramAddr) {
				case DVX_USB_RUN:
					return (spiConfigSend(&state->usbState, DVX_USB, paramAddr, param));
					break;

				case DVX_USB_EARLY_PACKET_DELAY: {
					// Early packet delay is 125µs slices on host, but in cycles
					// @ USB_CLOCK_FREQ on FPGA, so we must multiply here.
					float delayCC = roundf((float) param * 125.0F * state->deviceClocks.usbClockActual);
					return (spiConfigSend(&state->usbState, DVX_USB, paramAddr, U32T(delayCC)));
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP:
			switch (paramAddr) {
				case DVX_DVS_CHIP_MODE: {
					if (param >= 3) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_MODE, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_EVENT_FLATTEN: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT,
						(param) ? U8T(currVal | 0x40) : U8T(U8T(currVal) & ~0x40)));
					break;
				}

				case DVX_DVS_CHIP_EVENT_ON_ONLY: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT,
						(param) ? U8T(currVal | 0x20) : U8T(U8T(currVal) & ~0x20)));
					break;
				}

				case DVX_DVS_CHIP_EVENT_OFF_ONLY: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT,
						(param) ? U8T(currVal | 0x10) : U8T(U8T(currVal) & ~0x10)));
					break;
				}

				case DVX_DVS_CHIP_SUBSAMPLE_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE,
						(param) ? U8T(U8T(currVal) & ~0x04) : U8T(currVal | 0x04)));
					break;
				}

				case DVX_DVS_CHIP_AREA_BLOCKING_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE,
						(param) ? U8T(U8T(currVal) & ~0x02) : U8T(currVal | 0x02)));
					break;
				}

				case DVX_DVS_CHIP_DUAL_BINNING_ENABLE: {
					if (handle->info.chipID == DVXPLORER_LITE_CHIP_ID) {
						// Feature not available on LITE.
						return (false);
					}

					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_DUAL_BINNING, (param) ? (0x01) : (0x00)));
					break;
				}

				case DVX_DVS_CHIP_SUBSAMPLE_VERTICAL: {
					if (handle->info.chipID == DVXPLORER_LITE_CHIP_ID) {
						param = (param << 1) | 0x01;
					}

					if (param >= 8) {
						return (false);
					}

					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_SUBSAMPLE_RATIO, &currVal)) {
						return (false);
					}

					currVal = U8T(U8T(currVal) & ~0x38) | U8T(param << 3);

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_SUBSAMPLE_RATIO, currVal));
					break;
				}

				case DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL: {
					if (handle->info.chipID == DVXPLORER_LITE_CHIP_ID) {
						param = (param << 1) | 0x01;
					}

					if (param >= 8) {
						return (false);
					}

					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_SUBSAMPLE_RATIO, &currVal)) {
						return (false);
					}

					currVal = U8T(U8T(currVal) & ~0x07) | U8T(param);

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_SUBSAMPLE_RATIO, currVal));
					break;
				}

				case DVX_DVS_CHIP_AREA_BLOCKING_0:
				case DVX_DVS_CHIP_AREA_BLOCKING_1:
				case DVX_DVS_CHIP_AREA_BLOCKING_2:
				case DVX_DVS_CHIP_AREA_BLOCKING_3:
				case DVX_DVS_CHIP_AREA_BLOCKING_4:
				case DVX_DVS_CHIP_AREA_BLOCKING_5:
				case DVX_DVS_CHIP_AREA_BLOCKING_6:
				case DVX_DVS_CHIP_AREA_BLOCKING_7:
				case DVX_DVS_CHIP_AREA_BLOCKING_8:
				case DVX_DVS_CHIP_AREA_BLOCKING_9:
				case DVX_DVS_CHIP_AREA_BLOCKING_10:
				case DVX_DVS_CHIP_AREA_BLOCKING_11:
				case DVX_DVS_CHIP_AREA_BLOCKING_12:
				case DVX_DVS_CHIP_AREA_BLOCKING_13:
				case DVX_DVS_CHIP_AREA_BLOCKING_14:
				case DVX_DVS_CHIP_AREA_BLOCKING_15:
				case DVX_DVS_CHIP_AREA_BLOCKING_16:
				case DVX_DVS_CHIP_AREA_BLOCKING_17:
				case DVX_DVS_CHIP_AREA_BLOCKING_18:
				case DVX_DVS_CHIP_AREA_BLOCKING_19: {
					uint16_t regAddr
						= REGISTER_DIGITAL_AREA_BLOCK + U16T(2 * (paramAddr - DVX_DVS_CHIP_AREA_BLOCKING_0));

					if (!spiConfigSend(&state->usbState, DEVICE_DVS, regAddr, U8T(param >> 8))) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, U16T(regAddr + 1), U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMESTAMP_RESET: {
					if (param) {
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_TIMESTAMP_RESET, 0x01);
						return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_TIMESTAMP_RESET, 0x00));
					}
					break;
				}

				case DVX_DVS_CHIP_GLOBAL_RESET_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL,
						(param) ? U8T(currVal | 0x02) : U8T(U8T(currVal) & ~0x02)));
					break;
				}

				case DVX_DVS_CHIP_GLOBAL_RESET_DURING_READOUT: {
					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_GLOBAL_RESET_READOUT,
						(param) ? (0x01) : (0x00)));
					break;
				}

				case DVX_DVS_CHIP_GLOBAL_HOLD_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL,
						(param) ? U8T(currVal | 0x01) : U8T(U8T(currVal) & ~0x01)));
					break;
				}

				case DVX_DVS_CHIP_FIXED_READ_TIME_ENABLE: {
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_FIXED_READ_TIME, (param) ? (0x01) : (0x00)));
					break;
				}

				case DVX_DVS_CHIP_EXTERNAL_TRIGGER_MODE: {
					if (param >= 3) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_EXTERNAL_TRIGGER, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_ED: {
					// TODO: figure this out.
					if (param >= 128000) {
						return (false);
					}

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT, 0x00);
					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT + 1, 0x00);
					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT + 2, 0x02));
					break;
				}

				case DVX_DVS_CHIP_TIMING_GH2GRS: {
					// TODO: figure this out.
					if (param >= 128000) {
						return (false);
					}

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT, 0x00);
					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT + 1, 0x00);
					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT + 2, 0x00));
					break;
				}

				case DVX_DVS_CHIP_TIMING_GRS: {
					// TODO: figure this out.
					if (param >= 128000) {
						return (false);
					}

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END, 0x00);
					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END + 1, 0x00);
					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END + 2, 0x01));
					break;
				}

				case DVX_DVS_CHIP_TIMING_GH2SEL: {
					if (param >= 256) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_FIRST_SELX_START, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_SELW: {
					if (param >= 256) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_SELX_WIDTH, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2AY_R: {
					if (param >= 256) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_AY_START, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2AY_F: {
					if (param >= 256) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_AY_END, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2R_R: {
					if (param >= 256) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_R_START, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2R_F: {
					if (param >= 256) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_R_END, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_NEXT_SEL: {
					if ((param >= 65536) || (param < 5)) {
						return (false);
					}

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_NEXT_SELX_START, U8T(param >> 8));
					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_NEXT_SELX_START + 1, U8T(param));

					// Also set MAX_EVENT_NUM, which is defined as NEXT_SEL-5, up to a maximum of 60.
					uint8_t maxEventNum = (param < 65) ? U8T(param - 5) : U8T(60);

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_MAX_EVENT_NUM, maxEventNum));
					break;
				}

				case DVX_DVS_CHIP_TIMING_NEXT_GH: {
					if (param >= 128) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_NEXT_GH_CNT, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_TIMING_READ_FIXED: {
					if (param >= 65536) {
						return (false);
					}

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_TIMING_READ_TIME_INTERVAL, U8T(param >> 8));
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_TIMING_READ_TIME_INTERVAL + 1, U8T(param)));
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP_CROPPER:
			switch (paramAddr) {
				case DVX_DVS_CHIP_CROPPER_ENABLE: {
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_CROPPER_BYPASS, (param) ? (0x00) : (0x01)));
					break;
				}

				case DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS:
				case DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS: {
					if (param >= U32T(handle->info.dvsSizeY)) {
						return (false);
					}

					if (state->dvs.flipY) {
						param = U32T(handle->info.dvsSizeY) - 1 - param;
					}

					// Cropper has a special corner case:
					// if both start and end pixels are in the same group,
					// only the mask in the END register is actually applied.
					// We must track the addresses, detect this, and properly
					// update all the masks as needed.
					if (paramAddr == DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS) {
						if (state->dvs.flipY) {
							state->dvs.cropperYEnd = U16T(param);
						}
						else {
							state->dvs.cropperYStart = U16T(param);
						}
					}

					if (paramAddr == DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS) {
						if (state->dvs.flipY) {
							state->dvs.cropperYStart = U16T(param);
						}
						else {
							state->dvs.cropperYEnd = U16T(param);
						}
					}

					uint8_t startGroup = U8T(state->dvs.cropperYStart / 8);
					uint8_t startMask  = U8T(0x00FF << (state->dvs.cropperYStart % 8));

					uint8_t endGroup = U8T(state->dvs.cropperYEnd / 8);
					uint8_t endMask  = U8T(0x00FF >> (7 - (state->dvs.cropperYEnd % 8)));

					if (startGroup == endGroup) {
						// EndMask must let pass only the unblocked pixels (set to 1).
						endMask = startMask & endMask;

						// StartMask doesn't matter in this case, reset to 0xFF.
						startMask = 0xFF;
					}

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_Y_START_GROUP, startGroup);
					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_Y_START_MASK, startMask);

					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_Y_END_GROUP, endGroup);
					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_Y_END_MASK, endMask));
					break;
				}

				case DVX_DVS_CHIP_CROPPER_X_START_ADDRESS: {
					if (param >= U32T(handle->info.dvsSizeX)) {
						return (false);
					}

					if (state->dvs.flipX) {
						param = U32T(handle->info.dvsSizeX) - 1 - param;

						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_END_ADDRESS, U8T(param >> 8));
						return (spiConfigSend(
							&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_END_ADDRESS + 1, U8T(param)));
					}
					else {
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_START_ADDRESS, U8T(param >> 8));
						return (spiConfigSend(
							&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_START_ADDRESS + 1, U8T(param)));
					}
					break;
				}

				case DVX_DVS_CHIP_CROPPER_X_END_ADDRESS: {
					if (param >= U32T(handle->info.dvsSizeX)) {
						return (false);
					}

					if (state->dvs.flipX) {
						param = U32T(handle->info.dvsSizeX) - 1 - param;

						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_START_ADDRESS, U8T(param >> 8));
						return (spiConfigSend(
							&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_START_ADDRESS + 1, U8T(param)));
					}
					else {
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_END_ADDRESS, U8T(param >> 8));
						return (spiConfigSend(
							&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_END_ADDRESS + 1, U8T(param)));
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP_ACTIVITY_DECISION:
			switch (paramAddr) {
				case DVX_DVS_CHIP_ACTIVITY_DECISION_ENABLE: {
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_BYPASS, (param) ? (0x00) : (0x01)));
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_POS_THRESHOLD: {
					if (param >= 65536) {
						return (false);
					}

					spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_THRESHOLD, U8T(param >> 8));
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_THRESHOLD + 1, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_NEG_THRESHOLD: {
					if (param >= 65536) {
						return (false);
					}

					spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_NEG_THRESHOLD, U8T(param >> 8));
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_NEG_THRESHOLD + 1, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_DEC_RATE: {
					if (param >= 16) {
						return (false);
					}

					return (
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_DEC_RATE, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_DEC_TIME: {
					if (param >= 32) {
						return (false);
					}

					return (
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_DEC_TIME, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_POS_MAX_COUNT: {
					if (param >= 65536) {
						return (false);
					}

					spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_MAX_COUNT, U8T(param >> 8));
					return (spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_MAX_COUNT + 1, U8T(param)));
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP_BIAS:
			switch (paramAddr) {
				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_LOG: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST,
						(param) ? U8T(currVal | 0x08) : U8T(U8T(currVal) & ~0x08)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_SF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST,
						(param) ? U8T(currVal | 0x04) : U8T(U8T(currVal) & ~0x04)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_ON: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST,
						(param) ? U8T(currVal | 0x02) : U8T(U8T(currVal) & ~0x02)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_nRST: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST,
						(param) ? U8T(currVal | 0x01) : U8T(U8T(currVal) & ~0x01)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_LOGA: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS,
							REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR, &currVal)) {
						return (false);
					}

					return (
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR,
							(param) ? U8T(currVal | 0x10) : U8T(U8T(currVal) & ~0x10)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_LOGD: {
					if (param >= 4) {
						return (false);
					}

					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS,
							REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR, &currVal)) {
						return (false);
					}

					return (
						spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR,
							U8T((U8T(currVal) & ~0x0C) | U8T(param << 2))));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_LEVEL_SF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF,
						(param) ? U8T(currVal | 0x10) : U8T(U8T(currVal) & ~0x10)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_LEVEL_nOFF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, &currVal)) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF,
						(param) ? U8T(currVal | 0x02) : U8T(U8T(currVal) & ~0x02)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_AMP: {
					if (param >= 9) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_AMP, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_ON: {
					if (param >= 9) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_OFF: {
					if (param >= 9) {
						return (false);
					}

					return (spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, U8T(param)));
					break;
				}

				case DVX_DVS_CHIP_BIAS_SIMPLE:
					spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_AMP, 0x04);
					spiConfigSend(
						&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR, 0x14);

					switch (param) {
						case DVX_DVS_CHIP_BIAS_SIMPLE_VERY_LOW: {
							spiConfigSend(
								&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x06);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7D);

							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, 0x06);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, 0x02);
							break;
						}

						case DVX_DVS_CHIP_BIAS_SIMPLE_LOW: {
							spiConfigSend(
								&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x06);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7D);

							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, 0x03);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, 0x05);
							break;
						}

						case DVX_DVS_CHIP_BIAS_SIMPLE_HIGH: {
							spiConfigSend(
								&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x04);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7F);

							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, 0x05);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, 0x03);
							break;
						}

						case DVX_DVS_CHIP_BIAS_SIMPLE_VERY_HIGH: {
							spiConfigSend(
								&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x04);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7F);

							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, 0x02);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, 0x06);
							break;
						}

						case DVX_DVS_CHIP_BIAS_SIMPLE_DEFAULT:
						default: {
							spiConfigSend(
								&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x06);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7D);

							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, 0x00);
							spiConfigSend(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, 0x08);
							break;
						}
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool dvXplorerConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;
	dvXplorerState state   = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			return (usbConfigGet(&state->usbState, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigGet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigGet(&state->container, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					*param = atomic_load(&state->deviceLogLevel);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_MUX:
			switch (paramAddr) {
				case DVX_MUX_RUN:
				case DVX_MUX_TIMESTAMP_RUN:
				case DVX_MUX_RUN_CHIP:
				case DVX_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DVX_MUX_DROP_DVS_ON_TRANSFER_STALL:
					return (spiConfigReceive(&state->usbState, DVX_MUX, paramAddr, param));
					break;

				case DVX_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DVX_MUX_STATISTICS_EXTINPUT_DROPPED:
				case DVX_MUX_STATISTICS_EXTINPUT_DROPPED + 1:
				case DVX_MUX_STATISTICS_DVS_DROPPED:
				case DVX_MUX_STATISTICS_DVS_DROPPED + 1:
					if (handle->info.muxHasStatistics) {
						return (spiConfigReceive(&state->usbState, DVX_MUX, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS:
			switch (paramAddr) {
				case DVX_DVS_RUN:
					return (spiConfigReceive(&state->usbState, DVX_DVS, paramAddr, param));
					break;

				case DVX_DVS_STATISTICS_COLUMN:
				case DVX_DVS_STATISTICS_COLUMN + 1:
				case DVX_DVS_STATISTICS_GROUP:
				case DVX_DVS_STATISTICS_GROUP + 1:
				case DVX_DVS_STATISTICS_DROPPED_COLUMN:
				case DVX_DVS_STATISTICS_DROPPED_COLUMN + 1:
				case DVX_DVS_STATISTICS_DROPPED_GROUP:
				case DVX_DVS_STATISTICS_DROPPED_GROUP + 1:
					if (handle->info.dvsHasStatistics) {
						return (spiConfigReceive(&state->usbState, DVX_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_IMU:
			switch (paramAddr) {
				case DVX_IMU_RUN_ACCELEROMETER:
				case DVX_IMU_RUN_GYROSCOPE:
				case DVX_IMU_RUN_TEMPERATURE:
				case DVX_IMU_ACCEL_DATA_RATE:
				case DVX_IMU_ACCEL_FILTER:
				case DVX_IMU_ACCEL_RANGE:
				case DVX_IMU_GYRO_DATA_RATE:
				case DVX_IMU_GYRO_FILTER:
				case DVX_IMU_GYRO_RANGE:
					return (spiConfigReceive(&state->usbState, DVX_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVX_EXTINPUT:
			switch (paramAddr) {
				case DVX_EXTINPUT_RUN_DETECTOR:
				case DVX_EXTINPUT_DETECT_RISING_EDGES:
				case DVX_EXTINPUT_DETECT_FALLING_EDGES:
				case DVX_EXTINPUT_DETECT_PULSES:
				case DVX_EXTINPUT_DETECT_PULSE_POLARITY:
					return (spiConfigReceive(&state->usbState, DVX_EXTINPUT, paramAddr, param));
					break;

				case DVX_EXTINPUT_DETECT_PULSE_LENGTH: {
					// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must divide here.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(&state->usbState, DVX_EXTINPUT, paramAddr, &cyclesValue)) {
						return (false);
					}

					float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
					*param        = U32T(delayCC);

					return (true);
					break;
				}

				case DVX_EXTINPUT_RUN_GENERATOR:
				case DVX_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DVX_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DVX_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigReceive(&state->usbState, DVX_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DVX_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DVX_EXTINPUT_GENERATE_PULSE_LENGTH: {
					if (handle->info.extInputHasGenerator) {
						// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
						// on FPGA, so we must divide here.
						uint32_t cyclesValue = 0;
						if (!spiConfigReceive(&state->usbState, DVX_EXTINPUT, paramAddr, &cyclesValue)) {
							return (false);
						}

						float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
						*param        = U32T(delayCC);

						return (true);
					}
					else {
						return (false);
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_SYSINFO:
			// No SystemInfo parameters can ever be get! Use the info struct!
			return (false);
			break;

		case DVX_USB:
			switch (paramAddr) {
				case DVX_USB_RUN:
					return (spiConfigReceive(&state->usbState, DVX_USB, paramAddr, param));
					break;

				case DVX_USB_EARLY_PACKET_DELAY: {
					// Early packet delay is 125µs slices on host, but in cycles
					// @ USB_CLOCK_FREQ on FPGA, so we must divide here.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(&state->usbState, DVX_USB, paramAddr, &cyclesValue)) {
						return (false);
					}

					float delayCC = roundf((float) cyclesValue / (125.0F * state->deviceClocks.usbClockActual));
					*param        = U32T(delayCC);

					return (true);
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP:
			switch (paramAddr) {
				case DVX_DVS_CHIP_MODE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_MODE, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_EVENT_FLATTEN: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x40) != 0);
					break;
				}

				case DVX_DVS_CHIP_EVENT_ON_ONLY: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x20) != 0);
					break;
				}

				case DVX_DVS_CHIP_EVENT_OFF_ONLY: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CONTROL_PACKET_FORMAT, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x10) != 0);
					break;
				}

				case DVX_DVS_CHIP_SUBSAMPLE_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x04) != 0);
					break;
				}

				case DVX_DVS_CHIP_AREA_BLOCKING_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x02) != 0);
					break;
				}

				case DVX_DVS_CHIP_DUAL_BINNING_ENABLE: {
					if (handle->info.chipID == DVXPLORER_LITE_CHIP_ID) {
						// Feature not available on LITE.
						return (false);
					}

					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_ENABLE, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_SUBSAMPLE_VERTICAL: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_SUBSAMPLE_RATIO, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x38) >> 3);

					if (handle->info.chipID == DVXPLORER_LITE_CHIP_ID) {
						*param = (*param >> 1);
					}
					break;
				}

				case DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_SUBSAMPLE_RATIO, &currVal)) {
						return (false);
					}

					*param = (currVal & 0x07);

					if (handle->info.chipID == DVXPLORER_LITE_CHIP_ID) {
						*param = (*param >> 1);
					}
					break;
				}

				case DVX_DVS_CHIP_AREA_BLOCKING_0:
				case DVX_DVS_CHIP_AREA_BLOCKING_1:
				case DVX_DVS_CHIP_AREA_BLOCKING_2:
				case DVX_DVS_CHIP_AREA_BLOCKING_3:
				case DVX_DVS_CHIP_AREA_BLOCKING_4:
				case DVX_DVS_CHIP_AREA_BLOCKING_5:
				case DVX_DVS_CHIP_AREA_BLOCKING_6:
				case DVX_DVS_CHIP_AREA_BLOCKING_7:
				case DVX_DVS_CHIP_AREA_BLOCKING_8:
				case DVX_DVS_CHIP_AREA_BLOCKING_9:
				case DVX_DVS_CHIP_AREA_BLOCKING_10:
				case DVX_DVS_CHIP_AREA_BLOCKING_11:
				case DVX_DVS_CHIP_AREA_BLOCKING_12:
				case DVX_DVS_CHIP_AREA_BLOCKING_13:
				case DVX_DVS_CHIP_AREA_BLOCKING_14:
				case DVX_DVS_CHIP_AREA_BLOCKING_15:
				case DVX_DVS_CHIP_AREA_BLOCKING_16:
				case DVX_DVS_CHIP_AREA_BLOCKING_17:
				case DVX_DVS_CHIP_AREA_BLOCKING_18:
				case DVX_DVS_CHIP_AREA_BLOCKING_19: {
					uint16_t regAddr
						= REGISTER_DIGITAL_AREA_BLOCK + U16T(2 * (paramAddr - DVX_DVS_CHIP_AREA_BLOCKING_0));

					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, regAddr, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, U16T(regAddr + 1), &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMESTAMP_RESET: {
					*param = false;
					break;
				}

				case DVX_DVS_CHIP_GLOBAL_RESET_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x02) != 0);
					break;
				}

				case DVX_DVS_CHIP_GLOBAL_RESET_DURING_READOUT: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_GLOBAL_RESET_READOUT, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_GLOBAL_HOLD_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_MODE_CONTROL, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x01) == true);
					break;
				}

				case DVX_DVS_CHIP_FIXED_READ_TIME_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_FIXED_READ_TIME, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_EXTERNAL_TRIGGER_MODE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_DIGITAL_EXTERNAL_TRIGGER, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_ED: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT + 1, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT + 2, &currVal)) {
						return (false);
					}

					*param |= currVal;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GH_COUNT, &currVal)) {
						return (false);
					}

					*param *= currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_GH2GRS: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT + 1, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT + 2, &currVal)) {
						return (false);
					}

					*param |= currVal;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_COUNT, &currVal)) {
						return (false);
					}

					*param *= currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_GRS: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END + 1, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END + 2, &currVal)) {
						return (false);
					}

					*param |= currVal;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_GRS_END, &currVal)) {
						return (false);
					}

					*param *= currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_GH2SEL: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_FIRST_SELX_START, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_SELW: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_SELX_WIDTH, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2AY_R: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_AY_START, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2AY_F: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_AY_END, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2R_R: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_R_START, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_SEL2R_F: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_R_END, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_NEXT_SEL: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_NEXT_SELX_START, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_TIMING_NEXT_SELX_START + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_NEXT_GH: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_NEXT_GH_CNT, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_TIMING_READ_FIXED: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_TIMING_READ_TIME_INTERVAL, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_TIMING_READ_TIME_INTERVAL + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP_CROPPER:
			switch (paramAddr) {
				case DVX_DVS_CHIP_CROPPER_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_BYPASS, &currVal)) {
						return (false);
					}

					*param = (currVal == 0) ? (1) : (0);
					break;
				}

				case DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS: {
					*param = state->dvs.cropperYStart;
					break;
				}

				case DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS: {
					*param = state->dvs.cropperYEnd;
					break;
				}

				case DVX_DVS_CHIP_CROPPER_X_START_ADDRESS: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_START_ADDRESS, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_START_ADDRESS + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				case DVX_DVS_CHIP_CROPPER_X_END_ADDRESS: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_END_ADDRESS, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_CROPPER_X_END_ADDRESS + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP_ACTIVITY_DECISION:
			switch (paramAddr) {
				case DVX_DVS_CHIP_ACTIVITY_DECISION_ENABLE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_BYPASS, &currVal)) {
						return (false);
					}

					*param = (currVal == 0) ? (1) : (0);
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_POS_THRESHOLD: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_THRESHOLD, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_THRESHOLD + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_NEG_THRESHOLD: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_NEG_THRESHOLD, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_NEG_THRESHOLD + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_DEC_RATE: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_DEC_RATE, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_DEC_TIME: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_DEC_TIME, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_ACTIVITY_DECISION_POS_MAX_COUNT: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_MAX_COUNT, &currVal)) {
						return (false);
					}

					*param = U32T(currVal << 8);

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_ACTIVITY_DECISION_POS_MAX_COUNT + 1, &currVal)) {
						return (false);
					}

					*param |= currVal;
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVX_DVS_CHIP_BIAS:
			switch (paramAddr) {
				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_LOG: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x08) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_SF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x04) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_ON: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x02) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_nRST: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(
							&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x01) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_LOGA: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS,
							REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x10) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_RANGE_LOGD: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS,
							REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x0C) >> 2);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_LEVEL_SF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x10) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_LEVEL_nOFF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, &currVal)) {
						return (false);
					}

					*param = ((currVal & 0x02) != 0);
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_AMP: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_AMP, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_ON: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_ON, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				case DVX_DVS_CHIP_BIAS_CURRENT_OFF: {
					uint32_t currVal = 0;

					if (!spiConfigReceive(&state->usbState, DEVICE_DVS, REGISTER_BIAS_CURRENT_OFF, &currVal)) {
						return (false);
					}

					*param = currVal;
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool dvXplorerDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;
	dvXplorerState state   = &handle->state;

	usbSetShutdownCallback(&state->usbState, dataShutdownNotify, dataShutdownUserPtr);

	// Store new data available/not available anymore call-backs.
	dataExchangeSetNotify(&state->dataExchange, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr);

	containerGenerationCommitTimestampReset(&state->container);

	if (!dataExchangeBufferInit(&state->dataExchange)) {
		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	if (!containerGenerationAllocate(&state->container, DVXPLORER_EVENT_TYPES)) {
		freeAllDataMemory(state);

		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPackets.polarity
		= caerPolarityEventPacketAllocate(DVXPLORER_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.polarity == NULL) {
		freeAllDataMemory(state);

		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentPackets.special
		= caerSpecialEventPacketAllocate(DVXPLORER_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.special == NULL) {
		freeAllDataMemory(state);

		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	state->currentPackets.imu6
		= caerIMU6EventPacketAllocate(DVXPLORER_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.imu6 == NULL) {
		freeAllDataMemory(state);

		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
		return (false);
	}

	// Ignore multi-part events (IMU) at startup, so that any initial
	// incomplete event is ignored. The START events reset this as soon as
	// the first one is observed.
	state->imu.ignoreEvents = true;

	// Ensure no data is left over from previous runs, if the camera
	// wasn't shut-down properly. First ensure it is shut down completely.
	dvXplorerConfigSet(cdh, DVX_DVS, DVX_DVS_RUN, false);
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, false);
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_GYROSCOPE, false);
	dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_TEMPERATURE, false);
	dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_RUN_DETECTOR, false);

	dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_RUN, false);
	dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_TIMESTAMP_RUN, false);
	dvXplorerConfigSet(cdh, DVX_USB, DVX_USB_RUN, false);

	// Then wait 10ms for FPGA device side buffers to clear.
	struct timespec clearSleep = {.tv_sec = 0, .tv_nsec = 10000000};
	thrd_sleep(&clearSleep, NULL);

	// And reset the USB side of things.
	usbControlResetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (dataExchangeStartProducers(&state->dataExchange)) {
		// Enable data transfer on USB end-point 2.
		dvXplorerConfigSet(cdh, DVX_USB, DVX_USB_RUN, true);
		dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_TIMESTAMP_RUN, true);
		dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_RUN, true);

		// Wait 50 ms for data transfer to be ready.
		struct timespec noDataSleep = {.tv_sec = 0, .tv_nsec = 50000000};
		thrd_sleep(&noDataSleep, NULL);

		dvXplorerConfigSet(cdh, DVX_DVS, DVX_DVS_RUN, true);
		dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, true);
		dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_GYROSCOPE, true);
		dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_TEMPERATURE, true);
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_RUN_DETECTOR, true);

		// Enable streaming from DVS chip.
		dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_MODE, DVX_DVS_CHIP_MODE_STREAM);
	}

	return (true);
}

bool dvXplorerDataStop(caerDeviceHandle cdh) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;
	dvXplorerState state   = &handle->state;

	if (dataExchangeStopProducers(&state->dataExchange)) {
		// Disable streaming from DVS chip.
		dvXplorerConfigSet(cdh, DVX_DVS_CHIP, DVX_DVS_CHIP_MODE, DVX_DVS_CHIP_MODE_OFF);

		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		dvXplorerConfigSet(cdh, DVX_DVS, DVX_DVS_RUN, false);
		dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, false);
		dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_GYROSCOPE, false);
		dvXplorerConfigSet(cdh, DVX_IMU, DVX_IMU_RUN_TEMPERATURE, false);
		dvXplorerConfigSet(cdh, DVX_EXTINPUT, DVX_EXTINPUT_RUN_DETECTOR, false);

		dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_RUN, false);
		dvXplorerConfigSet(cdh, DVX_MUX, DVX_MUX_TIMESTAMP_RUN, false);
		dvXplorerConfigSet(cdh, DVX_USB, DVX_USB_RUN, false);
	}

	usbDataTransfersStop(&state->usbState);

	dataExchangeBufferEmpty(&state->dataExchange);

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPackets.polarityPosition = 0;
	state->currentPackets.specialPosition  = 0;
	state->currentPackets.imu6Position     = 0;

	// Reset private composite events.
	memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

	return (true);
}

caerEventPacketContainer dvXplorerDataGet(caerDeviceHandle cdh) {
	dvXplorerHandle handle = (dvXplorerHandle) cdh;
	dvXplorerState state   = &handle->state;

	return (dataExchangeGet(&state->dataExchange, &state->usbState.dataTransfersRun));
}

#define TS_WRAP_ADD 0x8000

static inline bool ensureSpaceForEvents(
	caerEventPacketHeader *packet, size_t position, size_t numEvents, dvXplorerHandle handle) {
	if ((position + numEvents) <= (size_t) caerEventPacketHeaderGetEventCapacity(*packet)) {
		return (true);
	}

	caerEventPacketHeader grownPacket
		= caerEventPacketGrow(*packet, caerEventPacketHeaderGetEventCapacity(*packet) * 2);
	if (grownPacket == NULL) {
		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to grow event packet of type %d.",
			caerEventPacketHeaderGetEventType(*packet));
		return (false);
	}

	*packet = grownPacket;
	return (true);
}

static void dvXplorerEventTranslator(void *vhd, const uint8_t *buffer, size_t bufferSize) {
	dvXplorerHandle handle = vhd;
	dvXplorerState state   = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!usbDataTransfersAreRunning(&state->usbState)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bufferSize & 0x01) != 0) {
		dvXplorerLog(CAER_LOG_ALERT, handle, "%zu bytes received via USB, which is not a multiple of two.", bufferSize);
		bufferSize &= ~((size_t) 0x01);
	}

	for (size_t bufferPos = 0; bufferPos < bufferSize; bufferPos += 2) {
		// Allocate new packets for next iteration as needed.
		if (!containerGenerationAllocate(&state->container, DVXPLORER_EVENT_TYPES)) {
			dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
			return;
		}

		if (state->currentPackets.special == NULL) {
			state->currentPackets.special = caerSpecialEventPacketAllocate(
				DVXPLORER_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.special == NULL) {
				dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		}

		if (state->currentPackets.polarity == NULL) {
			state->currentPackets.polarity = caerPolarityEventPacketAllocate(
				DVXPLORER_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.polarity == NULL) {
				dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}

		if (state->currentPackets.imu6 == NULL) {
			state->currentPackets.imu6 = caerIMU6EventPacketAllocate(
				DVXPLORER_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.imu6 == NULL) {
				dvXplorerLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
				return;
			}
		}

		bool tsReset   = false;
		bool tsBigWrap = false;

		uint16_t event = le16toh(*((const uint16_t *) (&buffer[bufferPos])));

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			handleTimestampUpdateNewLogic(&state->timestamps, event, handle->info.deviceString, &state->deviceLogLevel);

			containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code  = U8T((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							dvXplorerLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							handleTimestampResetNewLogic(
								&state->timestamps, handle->info.deviceString, &state->deviceLogLevel);

							containerGenerationCommitTimestampReset(&state->container);
							containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							// Update Master/Slave status on incoming TS resets.
							// Async call to not deadlock here.
							spiConfigReceiveAsync(&state->usbState, DVX_SYSINFO, DVX_SYSINFO_DEVICE_IS_MASTER,
								&dvXplorerTSMasterStatusUpdater, &handle->info);

							break;
						}

						case 2: { // External input (falling edge)
							dvXplorerLog(CAER_LOG_DEBUG, handle, "External input (falling edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_FALLING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 3: { // External input (rising edge)
							dvXplorerLog(CAER_LOG_DEBUG, handle, "External input (rising edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_RISING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 4: { // External input (pulse)
							dvXplorerLog(CAER_LOG_DEBUG, handle, "External input (pulse) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_PULSE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 5: { // IMU Start (6 axes)
							dvXplorerLog(CAER_LOG_DEBUG, handle, "IMU6 Start event received.");

							state->imu.ignoreEvents = false;
							state->imu.count        = 0;
							state->imu.type         = 0;

							memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

							break;
						}

						case 7: { // IMU End
							if (state->imu.ignoreEvents) {
								break;
							}
							dvXplorerLog(CAER_LOG_DEBUG, handle, "IMU End event received.");

							if (state->imu.count == IMU_TOTAL_COUNT) {
								// Timestamp at event-stream insertion point.
								caerIMU6EventSetTimestamp(&state->imu.currentEvent, state->timestamps.current);

								caerIMU6EventValidate(&state->imu.currentEvent, state->currentPackets.imu6);

								// IMU6 and APS operate on an internal event and copy that to the actual output
								// packet here, in the END state, for a reason: if a packetContainer, with all its
								// packets, is committed due to hitting any of the triggers that are not TS reset
								// or TS wrap-around related, like number of polarity events, the event in the packet
								// would be left incomplete, and the event in the new packet would be corrupted.
								// We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
								// just deleting that event, but these kinds of commits happen much more often and the
								// possible data loss would be too significant. So instead we keep a private event,
								// fill it, and then only copy it into the packet here in the END state, at which point
								// the whole event is ready and cannot be broken/corrupted in any way anymore.
								if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.imu6,
										(size_t) state->currentPackets.imu6Position, 1, handle)) {
									caerIMU6Event imuCurrentEvent = caerIMU6EventPacketGetEvent(
										state->currentPackets.imu6, state->currentPackets.imu6Position);
									memcpy(imuCurrentEvent, &state->imu.currentEvent, sizeof(struct caer_imu6_event));
									state->currentPackets.imu6Position++;
								}
							}
							else {
								dvXplorerLog(CAER_LOG_INFO, handle,
									"IMU End: failed to validate IMU sample count (%" PRIu8 "), discarding samples.",
									state->imu.count);
							}

							break;
						}

						case 16: { // External generator (falling edge)
							dvXplorerLog(CAER_LOG_DEBUG, handle, "External generator (falling edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_FALLING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 17: { // External generator (rising edge)
							dvXplorerLog(CAER_LOG_DEBUG, handle, "External generator (rising edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_RISING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						default:
							dvXplorerLog(
								CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.", data);
							break;
					}
					break;

				case 1: { // X column address. 10 bits (9 - 0) contain address, bit 11 Start of Frame marker.
					uint16_t columnAddr = data & 0x03FF;
					bool startOfFrame   = data & 0x0800;

					if (startOfFrame) {
						dvXplorerLog(CAER_LOG_DEBUG, handle, "Start of Frame column marker detected.");

						if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
								(size_t) state->currentPackets.specialPosition, 1, handle)) {
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EVENT_READOUT_START);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
						}
					}

					// Check range conformity.
					if (columnAddr >= state->dvs.sizeX) {
						dvXplorerLog(CAER_LOG_ALERT, handle,
							"DVS: X address out of range (0-%d): %" PRIu16 ", due to USB communication issue.",
							state->dvs.sizeX - 1, columnAddr);
						break; // Skip invalid X address (don't update lastX).
					}

					state->dvs.lastX = columnAddr;
					break;
				}

				case 2:
				case 3: { // 8-pixel group event presence and polarity.
						  // Code 2 is MGROUP Group 2 (SGROUP OFF), Code 3 is MGROUP Group 1 (SGROUP ON).
					if (!ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.polarity,
							(size_t) state->currentPackets.polarityPosition, 8, handle)) {
						break;
					}

					bool polarity  = ((data & 0x0100) == 0);
					uint16_t lastY = (code == 3) ? (state->dvs.lastYG1) : (state->dvs.lastYG2);

					for (uint16_t i = 0, mask = 0x0001; i < 8; i++, mask <<= 1) {
						// Check if event present first.
						if ((data & mask) == 0) {
							continue;
						}

						// Received event!
						caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
							state->currentPackets.polarity, state->currentPackets.polarityPosition);

						// Timestamp at event-stream insertion point.
						caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
						caerPolarityEventSetPolarity(currentPolarityEvent, polarity);
						if (state->dvs.invertXY) {
							caerPolarityEventSetX(currentPolarityEvent, lastY + i);
							caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastX);
						}
						else {
							caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastX);
							caerPolarityEventSetY(currentPolarityEvent, lastY + i);
						}
						caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
						state->currentPackets.polarityPosition++;
					}

					break;
				}

				case 4: {
					// Decode address.
					uint16_t group1Address = data & 0x003F;
					uint16_t group2Offset  = U16T(data >> 6) & 0x001F;
					uint16_t group2Address
						= (data & 0x0800) ? (group1Address - group2Offset) : (group1Address + group2Offset);

					// 8 pixels per group.
					group1Address *= 8;
					group2Address *= 8;

					// Check range conformity.
					if (group1Address >= state->dvs.sizeY) {
						dvXplorerLog(CAER_LOG_ALERT, handle,
							"DVS: Group1 Y address out of range (0-%d): %" PRIu16 ", due to USB communication issue.",
							state->dvs.sizeY - 1, group1Address);
						break; // Skip invalid G1 Y address (don't update lastYs).
					}

					if (group2Address >= state->dvs.sizeY) {
						dvXplorerLog(CAER_LOG_ALERT, handle,
							"DVS: Group2 Y address out of range (0-%d): %" PRIu16 ", due to USB communication issue.",
							state->dvs.sizeY - 1, group2Address);
						break; // Skip invalid G2 Y address (don't update lastYs).
					}

					state->dvs.lastYG1 = group1Address;
					state->dvs.lastYG2 = group2Address;

					break;
				}

				case 5: {
					// Misc 8bit data.
					uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
					uint8_t misc8Data = U8T(data & 0x00FF);

					switch (misc8Code) {
						case 0:
							if (state->imu.ignoreEvents) {
								break;
							}
							dvXplorerLog(CAER_LOG_DEBUG, handle, "IMU Data event (%" PRIu8 ") received.", misc8Data);

							// IMU data event.
							switch (state->imu.count) {
								case 0:
								case 2:
								case 4:
								case 6:
								case 8:
								case 10:
								case 12:
									state->imu.tmpData = misc8Data;
									break;

								case 1: {
									// X/Y axes are inverted due to IMU chip rotation.
									int16_t accelY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										accelY = I16T(-accelY);
									}
									caerIMU6EventSetAccelY(&state->imu.currentEvent, accelY / state->imu.accelScale);
									break;
								}

								case 3: {
									// X/Y axes are inverted due to IMU chip rotation.
									int16_t accelX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										accelX = I16T(-accelX);
									}
									caerIMU6EventSetAccelX(&state->imu.currentEvent, accelX / state->imu.accelScale);
									break;
								}

								case 5: {
									int16_t accelZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										accelZ = I16T(-accelZ);
									}
									caerIMU6EventSetAccelZ(&state->imu.currentEvent, accelZ / state->imu.accelScale);

									// IMU parser count depends on which data is present.
									if ((state->imu.type & IMU_TYPE_TEMP) == 0) {
										if (state->imu.type & IMU_TYPE_GYRO) {
											// No temperature, but gyro.
											state->imu.count = U8T(state->imu.count + 2);
										}
										else {
											// No others enabled.
											state->imu.count = U8T(state->imu.count + 8);
										}
									}
									break;
								}

								case 7: {
									// Temperature is signed. Formula for converting to °C:
									// (SIGNED_VAL / 512) + 23
									int16_t temp = I16T((state->imu.tmpData << 8) | misc8Data);
									caerIMU6EventSetTemp(&state->imu.currentEvent, (temp / 512.0F) + 23.0F);

									// IMU parser count depends on which data is present.
									if ((state->imu.type & IMU_TYPE_GYRO) == 0) {
										// No others enabled.
										state->imu.count = U8T(state->imu.count + 6);
									}
									break;
								}

								case 9: {
									// X/Y axes are inverted due to IMU chip rotation.
									int16_t gyroY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										gyroY = I16T(-gyroY);
									}
									caerIMU6EventSetGyroY(&state->imu.currentEvent, gyroY / state->imu.gyroScale);
									break;
								}

								case 11: {
									// X/Y axes are inverted due to IMU chip rotation.
									int16_t gyroX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										gyroX = I16T(-gyroX);
									}
									caerIMU6EventSetGyroX(&state->imu.currentEvent, gyroX / state->imu.gyroScale);
									break;
								}

								case 13: {
									int16_t gyroZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										gyroZ = I16T(-gyroZ);
									}
									caerIMU6EventSetGyroZ(&state->imu.currentEvent, gyroZ / state->imu.gyroScale);
									break;
								}

								default:
									dvXplorerLog(CAER_LOG_ERROR, handle, "Got invalid IMU update sequence.");
									break;
							}

							state->imu.count++;

							break;

						case 3: {
							if (state->imu.ignoreEvents) {
								break;
							}
							dvXplorerLog(
								CAER_LOG_DEBUG, handle, "IMU Scale Config event (%" PRIu16 ") received.", data);

							// Set correct IMU accel and gyro scales, used to interpret subsequent
							// IMU samples from the device.
							state->imu.accelScale = calculateIMUAccelScale(U16T(data >> 3) & 0x03);
							state->imu.gyroScale  = calculateIMUGyroScale(data & 0x07);

							// Set expected type of data to come from IMU (accel, gyro, temp).
							state->imu.type = U8T(data >> 5) & 0x07;

							// IMU parser start count depends on which data is present.
							if (state->imu.type & IMU_TYPE_ACCEL) {
								// Accelerometer.
								state->imu.count = 0;
							}
							else if (state->imu.type & IMU_TYPE_TEMP) {
								// Temperature
								state->imu.count = 6;
							}
							else if (state->imu.type & IMU_TYPE_GYRO) {
								// Gyroscope.
								state->imu.count = 8;
							}
							else {
								// Nothing, should never happen.
								state->imu.count = 14;

								dvXplorerLog(CAER_LOG_ERROR, handle, "IMU Scale Config: no IMU sensors enabled.");
							}

							break;
						}

						default:
							dvXplorerLog(CAER_LOG_ERROR, handle, "Caught Misc8 event that can't be handled.");
							break;
					}

					break;
				}

				case 7: { // Timestamp wrap
					tsBigWrap = handleTimestampWrapNewLogic(
						&state->timestamps, data, TS_WRAP_ADD, handle->info.deviceString, &state->deviceLogLevel);

					if (tsBigWrap) {
						if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
								(size_t) state->currentPackets.specialPosition, 1, handle)) {
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
							caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
						}
					}
					else {
						containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
					}

					break;
				}

				default:
					dvXplorerLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// tsReset and tsBigWrap are already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = containerGenerationGetMaxPacketSize(&state->container);
		bool containerSizeCommit                 = (currentPacketContainerCommitSize > 0)
								   && ((state->currentPackets.polarityPosition >= currentPacketContainerCommitSize)
									   || (state->currentPackets.specialPosition >= currentPacketContainerCommitSize)
									   || (state->currentPackets.imu6Position >= currentPacketContainerCommitSize));

		bool containerTimeCommit = containerGenerationIsCommitTimestampElapsed(
			&state->container, state->timestamps.wrapOverflow, state->timestamps.current);

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentPackets.polarityPosition > 0) {
				containerGenerationSetPacket(
					&state->container, POLARITY_EVENT, (caerEventPacketHeader) state->currentPackets.polarity);

				state->currentPackets.polarity         = NULL;
				state->currentPackets.polarityPosition = 0;
				emptyContainerCommit                   = false;
			}

			if (state->currentPackets.specialPosition > 0) {
				containerGenerationSetPacket(
					&state->container, SPECIAL_EVENT, (caerEventPacketHeader) state->currentPackets.special);

				state->currentPackets.special         = NULL;
				state->currentPackets.specialPosition = 0;
				emptyContainerCommit                  = false;
			}

			if (state->currentPackets.imu6Position > 0) {
				containerGenerationSetPacket(
					&state->container, IMU6_EVENT_PKT_POS, (caerEventPacketHeader) state->currentPackets.imu6);

				state->currentPackets.imu6         = NULL;
				state->currentPackets.imu6Position = 0;
				emptyContainerCommit               = false;
			}

			if (tsReset || tsBigWrap) {
				// Ignore all IMU6 (composite) events, until a new IMU6
				// Start event comes in, for the next packet.
				// This is to correctly support the forced packet commits that a TS reset,
				// or a TS big wrap, impose. Continuing to parse events would result
				// in a corrupted state of the first event in the new packet, as it would
				// be incomplete, incorrect and miss vital initialization data.
				// See IMU6 END states for more details on a related issue.
				state->imu.ignoreEvents = true;
			}

			containerGenerationExecute(&state->container, emptyContainerCommit, tsReset, state->timestamps.wrapOverflow,
				state->timestamps.current, &state->dataExchange, &state->usbState.dataTransfersRun,
				handle->info.deviceID, handle->info.deviceString, &state->deviceLogLevel);
		}
	}
}

static void dvXplorerTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param) {
	// If any USB error happened, discard.
	if (status != LIBUSB_TRANSFER_COMPLETED) {
		return;
	}

	// Get new Master/Slave information from device.
	struct caer_dvx_info *info = userDataPtr;

	atomic_thread_fence(memory_order_seq_cst);
	info->deviceIsMaster = param;
	atomic_thread_fence(memory_order_seq_cst);
}

//////////////////////////////////
/// FX3 Debug Transfer Support ///
//////////////////////////////////
static void allocateDebugTransfers(dvXplorerHandle handle) {
	// Allocate transfers and set them up.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		handle->state.fx3Support.debugTransfers[i] = libusb_alloc_transfer(0);
		if (handle->state.fx3Support.debugTransfers[i] == NULL) {
			dvXplorerLog(CAER_LOG_CRITICAL, handle,
				"Unable to allocate further libusb transfers (debug channel, %zu of %" PRIu32 ").", i,
				DEBUG_TRANSFER_NUM);
			continue;
		}

		// Create data buffer.
		handle->state.fx3Support.debugTransfers[i]->length = DEBUG_TRANSFER_SIZE;
		handle->state.fx3Support.debugTransfers[i]->buffer = malloc(DEBUG_TRANSFER_SIZE);
		if (handle->state.fx3Support.debugTransfers[i]->buffer == NULL) {
			dvXplorerLog(CAER_LOG_CRITICAL, handle,
				"Unable to allocate buffer for libusb transfer %zu (debug channel). Error: %d.", i, errno);

			libusb_free_transfer(handle->state.fx3Support.debugTransfers[i]);
			handle->state.fx3Support.debugTransfers[i] = NULL;

			continue;
		}

		// Initialize Transfer.
		handle->state.fx3Support.debugTransfers[i]->dev_handle = handle->state.usbState.deviceHandle;
		handle->state.fx3Support.debugTransfers[i]->endpoint   = DEBUG_ENDPOINT;
		handle->state.fx3Support.debugTransfers[i]->type       = LIBUSB_TRANSFER_TYPE_INTERRUPT;
		handle->state.fx3Support.debugTransfers[i]->callback   = &libUsbDebugCallback;
		handle->state.fx3Support.debugTransfers[i]->user_data  = handle;
		handle->state.fx3Support.debugTransfers[i]->timeout    = 0;
		handle->state.fx3Support.debugTransfers[i]->flags      = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(handle->state.fx3Support.debugTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_fetch_add(&handle->state.fx3Support.activeDebugTransfers, 1);
		}
		else {
			dvXplorerLog(CAER_LOG_CRITICAL, handle,
				"Unable to submit libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(handle->state.fx3Support.debugTransfers[i]);
			handle->state.fx3Support.debugTransfers[i] = NULL;
		}
	}

	if (atomic_load(&handle->state.fx3Support.activeDebugTransfers) == 0) {
		// Didn't manage to allocate any USB transfers, log failure.
		dvXplorerLog(CAER_LOG_CRITICAL, handle, "Unable to allocate any libusb transfers (debug channel).");
	}
}

static void cancelAndDeallocateDebugTransfers(dvXplorerHandle handle) {
	// Wait for all transfers to go away.
	struct timespec waitForTerminationSleep = {.tv_sec = 0, .tv_nsec = 1000000};

	while (atomic_load(&handle->state.fx3Support.activeDebugTransfers) > 0) {
		// Continue trying to cancel all transfers until there are none left.
		// It seems like one cancel pass is not enough and some hang around.
		for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
			if (handle->state.fx3Support.debugTransfers[i] != NULL) {
				errno = libusb_cancel_transfer(handle->state.fx3Support.debugTransfers[i]);
				if ((errno != LIBUSB_SUCCESS) && (errno != LIBUSB_ERROR_NOT_FOUND)) {
					dvXplorerLog(CAER_LOG_CRITICAL, handle,
						"Unable to cancel libusb transfer %zu (debug channel). Error: %s (%d).", i,
						libusb_strerror(errno), errno);
					// Proceed with trying to cancel all transfers regardless of errors.
				}
			}
		}

		// Sleep for 1ms to avoid busy loop.
		thrd_sleep(&waitForTerminationSleep, NULL);
	}

	// No more transfers in flight, deallocate them all here.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		if (handle->state.fx3Support.debugTransfers[i] != NULL) {
			libusb_free_transfer(handle->state.fx3Support.debugTransfers[i]);
			handle->state.fx3Support.debugTransfers[i] = NULL;
		}
	}
}

static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer) {
	dvXplorerHandle handle = transfer->user_data;

	// Completed or cancelled transfers are what we expect to handle here, so
	// if they do have data attached, try to parse them.
	if (((transfer->status == LIBUSB_TRANSFER_COMPLETED) || (transfer->status == LIBUSB_TRANSFER_CANCELLED))
		&& (transfer->actual_length > 0)) {
		// Handle debug data.
		debugTranslator(handle, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter and exiting.
	// Freeing the transfers is taken care of by cancelAndDeallocateDebugTransfers().
	atomic_fetch_sub(&handle->state.fx3Support.activeDebugTransfers, 1);
}

static void debugTranslator(dvXplorerHandle handle, const uint8_t *buffer, size_t bytesSent) {
	// Check if this is a debug message (length 7-64 bytes).
	if ((bytesSent >= 7) && (buffer[0] == 0x00)) {
		// Debug message, log this.
		dvXplorerLog(CAER_LOG_ERROR, handle, "Error message: '%s' (code %u at time %u).", &buffer[6], buffer[1],
			*((const uint32_t *) &buffer[2]));
	}
	else {
		// Unknown/invalid debug message, log this.
		dvXplorerLog(CAER_LOG_WARNING, handle, "Unknown/invalid debug message.");
	}
}

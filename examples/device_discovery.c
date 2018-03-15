/*
*   Author: Carsten Nielsen, 2018
*  compile with:  gcc -std=c11 -pedantic -Wall -Wextra -O2 -o device_discovery device_discovery.c -D_DEFAULT_SOURCE=1 -lcaer
*
*  Prints a list of libcAER supported devices connected to the system.
*/

#include <libcaer/libcaer.h>
#include <libcaer/devices/dynapse.h>
#include <libcaer/devices/davis.h>
#include <libcaer/devices/dvs128.h>
#include <libcaer/devices/device.h>
#include <stdio.h>
#include <signal.h>
#include <stdatomic.h>
#include <unistd.h>

static atomic_bool globalShutdown = ATOMIC_VAR_INIT(false);

static void globalShutdownSignalHandler(int signal) {
	// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
	if (signal == SIGTERM || signal == SIGINT) {
		atomic_store(&globalShutdown, true);
	}
}

int main(void) {
	// Install signal handler for global shutdown.
#if defined(_WIN32)
	if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#else
	struct sigaction shutdownAction;

	shutdownAction.sa_handler = &globalShutdownSignalHandler;
	shutdownAction.sa_flags = 0;
	sigemptyset(&shutdownAction.sa_mask);
	sigaddset(&shutdownAction.sa_mask, SIGTERM);
	sigaddset(&shutdownAction.sa_mask, SIGINT);

	if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#endif

	// We create an array of info handles to contain the discovered devices.
	caerDeviceInfoHandle *handleList = NULL;

	// Passing the address of the array lets libcAER modify its contents.
	ssize_t numDevices = caerDeviceDiscover(&handleList);

	// caerDeviceDiscover returns the number of devices found or -1 on error.
	if (numDevices == -1) {
		printf("Failed to init libusb\n");
	}
	else {
		printf("found %ld devices\n", numDevices);
		// We can now iterate over the discovered devices and cast their info handles
		// to device specific handles from which we can obtain information about the device.
		for (ssize_t i = 0; i < numDevices; i++) {
			switch(handleList[i]->deviceType) {
			case CAER_DEVICE_DYNAPSE:
				{
				dynapseInfoHandle infoHandle = (dynapseInfoHandle)handleList[i];
				printf("DYNAPSE ID: %d SERIAL: %s BUS: %u DEVICE ADDR: %u LOGIC: %d CLOCK: %d\n",
				        infoHandle->info.deviceID, infoHandle->info.deviceSerialNumber,
					infoHandle->info.deviceUSBBusNumber, infoHandle->info.deviceUSBDeviceAddress,
					infoHandle->info.logicVersion, infoHandle->info.logicClock);
				break;
				}
			case CAER_DEVICE_DAVIS:
				{
				davisInfoHandle infoHandle = (davisInfoHandle)handleList[i];
				printf("DAVIS ID: %d SERIAL: %s BUS: %u DEVICE ADDR: %u LOGIC: %d CLOCK: %d\n",
				        infoHandle->info.deviceID, infoHandle->info.deviceSerialNumber,
					infoHandle->info.deviceUSBBusNumber, infoHandle->info.deviceUSBDeviceAddress,
					infoHandle->info.logicVersion, infoHandle->info.logicClock);
				break;
				}
			case CAER_DEVICE_DVS128:
				{
				dvs128InfoHandle infoHandle = (dvs128InfoHandle)handleList[i];
				printf("DVS128 ID: %d SERIAL: %s BUS: %u DEVICE ADDR: %u LOGIC: %d\n",
				       infoHandle->info.deviceID, infoHandle->info.deviceSerialNumber,
				       infoHandle->info.deviceUSBBusNumber, infoHandle->info.deviceUSBDeviceAddress,
				       infoHandle->info.logicVersion);
				break;
				}
			}
		}
	}

	return (EXIT_SUCCESS);
}

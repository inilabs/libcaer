/**
 * @file device.h
 *
 * Common functions to access, configure and exchange data with
 * supported devices. Also contains defines for host
 * related configuration options.
 */

#ifndef LIBCAER_DEVICES_DEVICE_H_
#define LIBCAER_DEVICES_DEVICE_H_

#include "../libcaer.h"
#include "../events/packetContainer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Pointer to an open device on which to operate.
 */
typedef struct caer_device_handle *caerDeviceHandle;

struct caer_device_info_handle {
	uint16_t deviceType;
};

/**
 * Pointer to device info on non-open device.
 */
typedef struct caer_device_info_handle *caerDeviceInfoHandle;

/**
 * Module address: host-side data exchange (ring-buffer) configuration.
 */
#define CAER_HOST_CONFIG_DATAEXCHANGE -2
/**
 * Module address: host-side event packets generation configuration.
 */
#define CAER_HOST_CONFIG_PACKETS -3
/**
 * Module address: host-side logging configuration.
 */
#define CAER_HOST_CONFIG_LOG -4

/**
 * Parameter address for module CAER_HOST_CONFIG_DATAEXCHANGE:
 * set size of elements that can be held by the thread-safe FIFO
 * buffer between the data transfer thread and the main thread.
 * The default values are usually fine, only change them if you're
 * running into lots of dropped/missing packets; you can turn on
 * the INFO log level to see when this is the case.
 */
#define CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE     0
/**
 * Parameter address for module CAER_HOST_CONFIG_DATAEXCHANGE:
 * when calling caerDeviceDataGet(), the function can either be
 * blocking, meaning it waits until it has a valid
 * EventPacketContainer to return, or not, meaning it returns
 * right away. This behavior can be set with this flag.
 * Please see the caerDeviceDataGet() documentation for more
 * information on its return values.
 */
#define CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING        1
/**
 * Parameter address for module CAER_HOST_CONFIG_DATAEXCHANGE:
 * whether to start all the data producer modules on the device
 * (DVS, APS, Mux, ...) automatically when starting the
 * data transfer thread with caerDeviceDataStart() or not.
 * If disabled, be aware you will have to start the right modules
 * manually, which can be useful if you need precise control
 * over which ones are running at any time.
 */
#define CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS 2
/**
 * Parameter address for module CAER_HOST_CONFIG_DATAEXCHANGE:
 * whether to stop all the data producer modules on the device
 * (DVS, APS, Mux, ...) automatically when stopping the
 * data transfer thread with caerDeviceDataStop() or not.
 * If disabled, be aware you will have to stop the right modules
 * manually, to halt the data flow, which can be useful if you
 * need precise control over which ones are running at any time.
 */
#define CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS  3

/**
 * Parameter address for module CAER_HOST_CONFIG_PACKETS:
 * set the maximum number of events any of a packet container's
 * packets may hold before it's made available to the user.
 * Set to zero to disable.
 * This is checked for each number of events held in each typed
 * EventPacket that is a part of the EventPacketContainer.
 */
#define CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE 0
/**
 * Parameter address for module CAER_HOST_CONFIG_PACKETS:
 * set the time interval between subsequent packet containers.
 * Must be at least 1 microsecond.
 * The value is in microseconds, and is checked across all
 * types of events contained in the EventPacketContainer.
 */
#define CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL    1

/**
 * Parameter address for module CAER_HOST_CONFIG_LOG:
 * set the log-level for this device, to be used when logging
 * messages. Defaults to the value of the global log-level
 * when the device was first opened.
 */
#define CAER_HOST_CONFIG_LOG_LEVEL 0

/**
 * Close a previously opened device and invalidate its handle.
 *
 * @param handle pointer to a valid device handle. Will set handle to NULL if closing is
 *               successful, to prevent further usage of this handle for other operations.
 *
 * @return true if closing was successful, false on errors.
 */
bool caerDeviceClose(caerDeviceHandle *handle);

/**
* Find supported devices currently connected, but not opened.
*
* @param pointer to array of caerDeviceInfoHandles. The array will be filled with info
* handle elements.
*
* @return the number of elements placed in the infoBuffer array, or -1 on error.
*/
ssize_t caerDeviceDiscover(caerDeviceInfoHandle **infoBuffer);

/**
 * Send a set of good default configuration settings to the device.
 * This avoids users having to set every configuration option each time,
 * especially when wanting to get going quickly or just needing to change
 * a few settings to get to the desired operating mode.
 *
 * @param handle a valid device handle.
 *
 * @return true if sending the configuration was successful, false on errors.
 */
bool caerDeviceSendDefaultConfig(caerDeviceHandle handle);

/**
 * Set a configuration parameter to a given value.
 *
 * @param handle a valid device handle.
 * @param modAddr a module address, used to specify which configuration module
 *                one wants to update. Negative addresses are used for host-side
 *                configuration, while positive addresses (including zero) are
 *                used for device-side configuration.
 * @param paramAddr a parameter address, to select a specific parameter to update
 *                  from this particular configuration module. Only positive numbers
 *                  (including zero) are allowed.
 * @param param a configuration parameter's new value.
 *
 * @return true if sending the configuration was successful, false on errors.
 */
bool caerDeviceConfigSet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param);

/**
 * Get the value of a configuration parameter.
 *
 * @param handle a valid device handle.
 * @param modAddr a module address, used to specify which configuration module
 *                one wants to query. Negative addresses are used for host-side
 *                configuration, while positive addresses (including zero) are
 *                used for device-side configuration.
 * @param paramAddr a parameter address, to select a specific parameter to query
 *                  from this particular configuration module. Only positive numbers
 *                  (including zero) are allowed.
 * @param param a pointer to an integer, in which to store the configuration
 *              parameter's current value. The integer will always be either set
 *              to zero (on failure), or to the current value (on success).
 *
 * @return true if getting the configuration was successful, false on errors.
 */
bool caerDeviceConfigGet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

/**
 * Get the value of a 64bit configuration parameter.
 * This is for special read-only configuration parameters only!
 * Use only when required by the parameter's documentation!
 *
 * @param handle a valid device handle.
 * @param modAddr a module address, used to specify which configuration module
 *                one wants to query. Negative addresses are used for host-side
 *                configuration, while positive addresses (including zero) are
 *                used for device-side configuration.
 * @param paramAddr a parameter address, to select a specific parameter to query
 *                  from this particular configuration module. Only positive numbers
 *                  (including zero) are allowed.
 * @param param a pointer to a 64bit integer, in which to store the configuration
 *              parameter's current value. The integer will always be either set
 *              to zero (on failure), or to the current value (on success).
 *
 * @return true if getting the configuration was successful, false on errors.
 */
bool caerDeviceConfigGet64(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint64_t *param);

/**
 * Start getting data from the device, setting up the data transfers
 * and starting the data producers (see CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS).
 * Supports notification of new data and exceptional shutdown events via user-defined call-backs.
 *
 * @param handle a valid device handle.
 * @param dataNotifyIncrease function pointer, called every time a new piece of data
 *                           available and has been put in the FIFO buffer for consumption.
 *                           dataNotifyUserPtr will be passed as parameter to the function.
 * @param dataNotifyDecrease function pointer, called every time a new piece of data
 *                           has been consumed from the FIFO buffer inside caerDeviceDataGet().
 *                           dataNotifyUserPtr will be passed as parameter to the function.
 * @param dataNotifyUserPtr pointer that will be passed to the dataNotifyIncrease and
 *                          dataNotifyDecrease functions. Can be NULL.
 * @param dataShutdownNotify function pointer, called on exceptional shut-down of the
 *                           data transfers. This is used to detect exceptional shut-downs
 *                           that do not come from calling caerDeviceDataStop(), such as
 *                           when the device is disconnected or all data transfers fail.
 * @param dataShutdownUserPtr pointer that will be passed to the dataShutdownNotify
 *                            function. Can be NULL.
 *
 * @return true if starting the data transfer was successful, false on errors.
 */
bool caerDeviceDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);

/**
 * Stop getting data from the device, shutting down the data transfers
 * and stopping the data producers (see CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS).
 * This normal shut-down will not generate a notification (see caerDeviceDataStart()).
 *
 * @param handle a valid device handle.
 *
 * @return true if stopping the data transfer was successful, false on errors.
 */
bool caerDeviceDataStop(caerDeviceHandle handle);

/**
 * Get an event packet container, which contains events of various types generated by
 * the device, for further processing.
 * The returned data structures are allocated in memory and will need to be freed.
 * The caerEventPacketContainerFree() function can be used to correctly free the full
 * container memory. For single caerEventPackets, just use free().
 * This function can be made blocking with the CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING
 * configuration parameter. By default it is non-blocking.
 *
 * @param handle a valid device handle.
 *
 * @return a valid event packet container. NULL will be returned on errors, such as
 *         exceptional device shutdown, or when there is no container available in
 *         non-blocking mode. Always check this return value!
 */
caerEventPacketContainer caerDeviceDataGet(caerDeviceHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_DEVICE_H_ */

/**
 * @file special.h
 *
 * Special Events format definition and handling functions.
 * This event type encodes special occurrences, such as
 * timestamp related notifications or external input events.
 */

#ifndef LIBCAER_EVENTS_SPECIAL_H_
#define LIBCAER_EVENTS_SPECIAL_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Shift and mask values for the type and data portions
 * of a special event.
 * Up to 128 types, with 24 bits of data each, are possible.
 * Bit 0 is the valid mark, see 'common.h' for more details.
 */
//@{
#define TYPE_SHIFT 1
#define TYPE_MASK 0x0000007F
#define DATA_SHIFT 8
#define DATA_MASK 0x00FFFFFF
//@}

/**
 * List of all special event type identifiers.
 * Used to interpret the special event type field.
 */
enum caer_special_event_types {
	TIMESTAMP_WRAP = 0,                //!< A 32 bit timestamp wrap occurred.
	TIMESTAMP_RESET = 1,               //!< A timestamp reset occurred.
	EXTERNAL_INPUT_RISING_EDGE = 2,    //!< A rising edge was detected (External Input module on device).
	EXTERNAL_INPUT_FALLING_EDGE = 3,   //!< A falling edge was detected (External Input module on device).
	EXTERNAL_INPUT_PULSE = 4,          //!< A pulse was detected (External Input module on device).
	DVS_ROW_ONLY = 5,                  //!< A DVS row-only event was detected (a row address without any following column addresses).
	EXTERNAL_INPUT1_RISING_EDGE = 6,   //!< A rising edge was detected (External Input 1 module on device).
	EXTERNAL_INPUT1_FALLING_EDGE = 7,  //!< A falling edge was detected (External Input 1 module on device).
	EXTERNAL_INPUT1_PULSE = 8,         //!< A pulse was detected (External Input 1 module on device).
	EXTERNAL_INPUT2_RISING_EDGE = 9,   //!< A rising edge was detected (External Input 2 module on device).
	EXTERNAL_INPUT2_FALLING_EDGE = 10, //!< A falling edge was detected (External Input 2 module on device).
	EXTERNAL_INPUT2_PULSE = 11,        //!< A pulse was detected (External Input 2 module on device).
	EXTERNAL_GENERATOR_RISING_EDGE = 12,  //!< A rising edge was generated (External Input Generator module on device).
	EXTERNAL_GENERATOR_FALLING_EDGE = 13, //!< A falling edge was generated (External Input Generator module on device).
	APS_FRAME_START = 14,              //!< An APS frame capture has started (Frame Event will follow).
	APS_FRAME_END = 15,                //!< An APS frame capture has completed (Frame Event is alongside).
	APS_EXPOSURE_START = 16,           //!< An APS frame exposure has started (Frame Event will follow).
	APS_EXPOSURE_END = 17,             //!< An APS frame exposure has completed (Frame Event will follow).
};

/**
 * Special event data structure definition.
 * This contains the actual data, as well as the 32 bit event timestamp.
 * Signed integers are used for fields that are to be interpreted
 * directly, for compatibility with languages that do not have
 * unsigned integer types, such as Java.
 */
PACKED_STRUCT(
struct caer_special_event {
	/// Event data. First because of valid mark.
	uint32_t data;
	/// Event timestamp.
	int32_t timestamp;
});

/**
 * Type for pointer to special event data structure.
 */
typedef struct caer_special_event *caerSpecialEvent;

/**
 * Special event packet data structure definition.
 * EventPackets are always made up of the common packet header,
 * followed by 'eventCapacity' events. Everything has to
 * be in one contiguous memory block.
 */
PACKED_STRUCT(
struct caer_special_event_packet {
	/// The common event packet header.
	struct caer_event_packet_header packetHeader;
	/// The events array.
	struct caer_special_event events[];
});

/**
 * Type for pointer to special event packet data structure.
 */
typedef struct caer_special_event_packet *caerSpecialEventPacket;

/**
 * Allocate a new special events packet.
 * Use free() to reclaim this memory.
 *
 * @param eventCapacity the maximum number of events this packet will hold.
 * @param eventSource the unique ID representing the source/generator of this packet.
 * @param tsOverflow the current timestamp overflow counter value for this packet.
 *
 * @return a valid SpecialEventPacket handle or NULL on error.
 */
caerSpecialEventPacket caerSpecialEventPacketAllocate(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow);

/**
 * Get the special event at the given index from the event packet.
 *
 * @param packet a valid SpecialEventPacket pointer. Cannot be NULL.
 * @param n the index of the returned event. Must be within [0,eventCapacity[ bounds.
 *
 * @return the requested special event. NULL on error.
 */
static inline caerSpecialEvent caerSpecialEventPacketGetEvent(caerSpecialEventPacket packet, int32_t n) {
	// Check that we're not out of bounds.
	if (n < 0 || n >= caerEventPacketHeaderGetEventCapacity(&packet->packetHeader)) {
		caerLog(CAER_LOG_CRITICAL, "Special Event",
			"Called caerSpecialEventPacketGetEvent() with invalid event offset %" PRIi32 ", while maximum allowed value is %" PRIi32 ".",
			n, caerEventPacketHeaderGetEventCapacity(&packet->packetHeader) - 1);
		return (NULL);
	}

	// Return a pointer to the specified event.
	return (packet->events + n);
}

/**
 * Get the 32bit event timestamp, in microseconds.
 * Be aware that this wraps around! You can either ignore this fact,
 * or handle the special 'TIMESTAMP_WRAP' event that is generated when
 * this happens, or use the 64bit timestamp which never wraps around.
 * See 'caerEventPacketHeaderGetEventTSOverflow()' documentation
 * for more details on the 64bit timestamp.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 *
 * @return this event's 32bit microsecond timestamp.
 */
static inline int32_t caerSpecialEventGetTimestamp(caerSpecialEvent event) {
	return (le32toh(event->timestamp));
}

/**
 * Get the 64bit event timestamp, in microseconds.
 * See 'caerEventPacketHeaderGetEventTSOverflow()' documentation
 * for more details on the 64bit timestamp.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 * @param packet the SpecialEventPacket pointer for the packet containing this event. Cannot be NULL.
 *
 * @return this event's 64bit microsecond timestamp.
 */
static inline int64_t caerSpecialEventGetTimestamp64(caerSpecialEvent event, caerSpecialEventPacket packet) {
	return (I64T(
		(U64T(caerEventPacketHeaderGetEventTSOverflow(&packet->packetHeader)) << TS_OVERFLOW_SHIFT) | U64T(caerSpecialEventGetTimestamp(event))));
}

/**
 * Set the 32bit event timestamp, the value has to be in microseconds.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 * @param timestamp a positive 32bit microsecond timestamp.
 */
static inline void caerSpecialEventSetTimestamp(caerSpecialEvent event, int32_t timestamp) {
	if (timestamp < 0) {
		// Negative means using the 31st bit!
		caerLog(CAER_LOG_CRITICAL, "Special Event", "Called caerSpecialEventSetTimestamp() with negative value!");
		return;
	}

	event->timestamp = htole32(timestamp);
}

/**
 * Check if this special event is valid.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 *
 * @return true if valid, false if not.
 */
static inline bool caerSpecialEventIsValid(caerSpecialEvent event) {
	return (GET_NUMBITS32(event->data, VALID_MARK_SHIFT, VALID_MARK_MASK));
}

/**
 * Validate the current event by setting its valid bit to true
 * and increasing the event packet's event count and valid
 * event count. Only works on events that are invalid.
 * DO NOT CALL THIS AFTER HAVING PREVIOUSLY ALREADY
 * INVALIDATED THIS EVENT, the total count will be incorrect.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 * @param packet the SpecialEventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerSpecialEventValidate(caerSpecialEvent event, caerSpecialEventPacket packet) {
	if (!caerSpecialEventIsValid(event)) {
		SET_NUMBITS32(event->data, VALID_MARK_SHIFT, VALID_MARK_MASK, 1);

		// Also increase number of events and valid events.
		// Only call this on (still) invalid events!
		caerEventPacketHeaderSetEventNumber(&packet->packetHeader,
			caerEventPacketHeaderGetEventNumber(&packet->packetHeader) + 1);
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) + 1);
	}
	else {
		caerLog(CAER_LOG_CRITICAL, "Special Event", "Called caerSpecialEventValidate() on already valid event.");
	}
}

/**
 * Invalidate the current event by setting its valid bit
 * to false and decreasing the number of valid events held
 * in the packet. Only works with events that are already
 * valid!
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 * @param packet the SpecialEventPacket pointer for the packet containing this event. Cannot be NULL.
 */
static inline void caerSpecialEventInvalidate(caerSpecialEvent event, caerSpecialEventPacket packet) {
	if (caerSpecialEventIsValid(event)) {
		CLEAR_NUMBITS32(event->data, VALID_MARK_SHIFT, VALID_MARK_MASK);

		// Also decrease number of valid events. Number of total events doesn't change.
		// Only call this on valid events!
		caerEventPacketHeaderSetEventValid(&packet->packetHeader,
			caerEventPacketHeaderGetEventValid(&packet->packetHeader) - 1);
	}
	else {
		caerLog(CAER_LOG_CRITICAL, "Special Event", "Called caerSpecialEventInvalidate() on already invalid event.");
	}
}

/**
 * Get the numerical special event type.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 *
 * @return the special event type (see 'enum caer_special_event_types').
 */
static inline uint8_t caerSpecialEventGetType(caerSpecialEvent event) {
	return U8T(GET_NUMBITS32(event->data, TYPE_SHIFT, TYPE_MASK));
}

/**
 * Set the numerical special event type.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 * @param type the special event type (see 'enum caer_special_event_types').
 */
static inline void caerSpecialEventSetType(caerSpecialEvent event, uint8_t type) {
	CLEAR_NUMBITS32(event->data, TYPE_SHIFT, TYPE_MASK);
	SET_NUMBITS32(event->data, TYPE_SHIFT, TYPE_MASK, type);
}

/**
 * Get the special event data. Its meaning depends on the type.
 * Current types that make use of it are (see 'enum caer_special_event_types'):
 * - DVS_ROW_ONLY: encodes the address of the row from the row-only event.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 *
 * @return the special event data.
 */
static inline uint32_t caerSpecialEventGetData(caerSpecialEvent event) {
	return U32T(GET_NUMBITS32(event->data, DATA_SHIFT, DATA_MASK));
}

/**
 * Set the special event data. Its meaning depends on the type.
 * Current types that make use of it are (see 'enum caer_special_event_types'):
 * - DVS_ROW_ONLY: encodes the address of the row from the row-only event.
 *
 * @param event a valid SpecialEvent pointer. Cannot be NULL.
 * @param data the special event data.
 */
static inline void caerSpecialEventSetData(caerSpecialEvent event, uint32_t data) {
	CLEAR_NUMBITS32(event->data, DATA_SHIFT, DATA_MASK);
	SET_NUMBITS32(event->data, DATA_SHIFT, DATA_MASK, data);
}

/**
 * Iterator over all special events in a packet.
 * Returns the current index in the 'caerSpecialIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerSpecialIteratorElement' variable
 * of type caerSpecialEvent.
 *
 * SPECIAL_PACKET: a valid SpecialEventPacket pointer. Cannot be NULL.
 */
#define CAER_SPECIAL_ITERATOR_ALL_START(SPECIAL_PACKET) \
	for (int32_t caerSpecialIteratorCounter = 0; \
		caerSpecialIteratorCounter < caerEventPacketHeaderGetEventNumber(&(SPECIAL_PACKET)->packetHeader); \
		caerSpecialIteratorCounter++) { \
		caerSpecialEvent caerSpecialIteratorElement = caerSpecialEventPacketGetEvent(SPECIAL_PACKET, caerSpecialIteratorCounter);

/**
 * Iterator close statement.
 */
#define CAER_SPECIAL_ITERATOR_ALL_END }

/**
 * Iterator over only the valid special events in a packet.
 * Returns the current index in the 'caerSpecialIteratorCounter' variable of type
 * 'int32_t' and the current event in the 'caerSpecialIteratorElement' variable
 * of type caerSpecialEvent.
 *
 * SPECIAL_PACKET: a valid SpecialEventPacket pointer. Cannot be NULL.
 */
#define CAER_SPECIAL_ITERATOR_VALID_START(SPECIAL_PACKET) \
	for (int32_t caerSpecialIteratorCounter = 0; \
		caerSpecialIteratorCounter < caerEventPacketHeaderGetEventNumber(&(SPECIAL_PACKET)->packetHeader); \
		caerSpecialIteratorCounter++) { \
		caerSpecialEvent caerSpecialIteratorElement = caerSpecialEventPacketGetEvent(SPECIAL_PACKET, caerSpecialIteratorCounter); \
		if (!caerSpecialEventIsValid(caerSpecialIteratorElement)) { continue; } // Skip invalid special events.

/**
 * Iterator close statement.
 */
#define CAER_SPECIAL_ITERATOR_VALID_END }

/**
 * Get the first special event with the given event type in this
 * event packet. This returns the first found event with that type ID,
 * or NULL if we get to the end without finding any such event.
 *
 * @param packet a valid SpecialEventPacket pointer. Cannot be NULL.
 * @param type the special event type to search for.
 *
 * @return the requested special event or NULL on error/not found.
 */
static inline caerSpecialEvent caerSpecialEventPacketFindEventByType(caerSpecialEventPacket packet, uint8_t type) {
	CAER_SPECIAL_ITERATOR_ALL_START(packet)
		if (caerSpecialEventGetType(caerSpecialIteratorElement) == type) {
			// Found it, return it.
			return (caerSpecialIteratorElement);
		}
	CAER_SPECIAL_ITERATOR_ALL_END

	// Found nothing, return nothing.
	return (NULL);
}

/**
 * Get the first valid special event with the given event type in this
 * event packet. This returns the first found valid event with that type ID,
 * or NULL if we get to the end without finding any such event.
 *
 * @param packet a valid SpecialEventPacket pointer. Cannot be NULL.
 * @param type the special event type to search for.
 *
 * @return the requested valid special event or NULL on error/not found.
 */
static inline caerSpecialEvent caerSpecialEventPacketFindValidEventByType(caerSpecialEventPacket packet, uint8_t type) {
	CAER_SPECIAL_ITERATOR_VALID_START(packet)
		if (caerSpecialEventGetType(caerSpecialIteratorElement) == type) {
			// Found it, return it.
			return (caerSpecialIteratorElement);
		}
	CAER_SPECIAL_ITERATOR_VALID_END

	// Found nothing, return nothing.
	return (NULL);
}

#undef TYPE_MASK

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_EVENTS_SPECIAL_H_ */

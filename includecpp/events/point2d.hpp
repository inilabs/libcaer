#ifndef LIBCAER_EVENTS_POINT2D_HPP_
#define LIBCAER_EVENTS_POINT2D_HPP_

#include <libcaer/events/point2d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class Point2DEventPacket: public EventPacketHeader {
public:
	using Point2DEventBase = struct caer_point2d_event;

	struct Point2DEvent: public Point2DEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerPoint2DEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const Point2DEventPacket &packet) const noexcept {
			return (caerPoint2DEventGetTimestamp64(this, reinterpret_cast<caerPoint2DEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerPoint2DEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerPoint2DEventIsValid(this));
		}

		void validate(Point2DEventPacket &packet) noexcept {
			caerPoint2DEventValidate(this, reinterpret_cast<caerPoint2DEventPacket>(packet.header));
		}

		void invalidate(Point2DEventPacket &packet) noexcept {
			caerPoint2DEventInvalidate(this, reinterpret_cast<caerPoint2DEventPacket>(packet.header));
		}

		uint8_t getType() const noexcept {
			return (caerPoint2DEventGetType(this));
		}

		void setType(uint8_t t) noexcept {
			return (caerPoint2DEventSetType(this, t));
		}

		int8_t getScale() const noexcept {
			return (caerPoint2DEventGetScale(this));
		}

		void setScale(int8_t s) noexcept {
			return (caerPoint2DEventSetScale(this, s));
		}

		float getX() const noexcept {
			return (caerPoint2DEventGetX(this));
		}

		void setX(float xVal) noexcept {
			return (caerPoint2DEventSetX(this, xVal));
		}

		float getY() const noexcept {
			return (caerPoint2DEventGetY(this));
		}

		void setY(float yVal) noexcept {
			return (caerPoint2DEventSetY(this, yVal));
		}
	};

	// Constructors.
	Point2DEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerPoint2DEventPacket packet = caerPoint2DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate Point2D event packet.");
		}

		header = &packet->packetHeader;
	}

	Point2DEventPacket(caerPoint2DEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != POINT2D_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	Point2DEventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != POINT2D_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	Point2DEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		Point2DEventBase *evtBase = caerPoint2DEventPacketGetEvent(reinterpret_cast<caerPoint2DEventPacket>(header),
			index);
		Point2DEvent *evt = static_cast<Point2DEvent *>(evtBase);

		return (*evt);
	}

	const Point2DEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const Point2DEventBase *evtBase = caerPoint2DEventPacketGetEventConst(
			reinterpret_cast<caerPoint2DEventPacketConst>(header), index);
		const Point2DEvent *evt = static_cast<const Point2DEvent *>(evtBase);

		return (*evt);
	}

	Point2DEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const Point2DEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}

	virtual Point2DEventPacket *copy() const override {
		return (new Point2DEventPacket(internalCopy(header)));
	}

	virtual Point2DEventPacket *copyOnlyEvents() const override {
		return (new Point2DEventPacket(internalCopyOnlyEvents(header)));
	}

	virtual Point2DEventPacket *copyOnlyValidEvents() const override {
		return (new Point2DEventPacket(internalCopyOnlyValidEvents(header)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_POINT2D_HPP_ */

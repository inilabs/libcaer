#ifndef LIBCAER_HPP_
#define LIBCAER_HPP_

#include <libcaer/libcaer.h>
#include <stdexcept>
#include <type_traits>

/*
namespace libcaer {
namespace log {

// Undefine the log-level names, to avoid conflicts with macros
// on Windows/MinGW for example.
#undef EMERGENCY
#undef ALERT
#undef CRITICAL
#undef ERROR
#undef WARNING
#undef NOTICE
#undef INFO
#undef DEBUG

enum class logLevel {
	EMERGENCY = 0,
	ALERT = 1,
	CRITICAL = 2,
	ERROR = 3,
	WARNING = 4,
	NOTICE = 5,
	INFO = 6,
	DEBUG = 7,
};

void logLevelSet(logLevel l) noexcept;
logLevel logLevelGet() noexcept;
void fileDescriptorsSet(int fd1, int fd2) noexcept;
int fileDescriptorsGetFirst() noexcept;
int fileDescriptorsGetSecond() noexcept;
void log(logLevel l, const char *subSystem, const char *format, ...) noexcept;
void logVA(logLevel l, const char *subSystem, const char *format, va_list args) noexcept;
void logVAFull(int logFileDescriptor1, int logFileDescriptor2, uint8_t systemLogLevel, logLevel l,
	const char *subSystem, const char *format, va_list args) noexcept;

void logLevelSet(logLevel l) noexcept {
	caerLogLevelSet(static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)));
}

logLevel logLevelGet() noexcept {
	return (static_cast<logLevel>(caerLogLevelGet()));
}

void fileDescriptorsSet(int fd1, int fd2) noexcept {
	caerLogFileDescriptorsSet(fd1, fd2);
}

int fileDescriptorsGetFirst() noexcept {
	return (caerLogFileDescriptorsGetFirst());
}

int fileDescriptorsGetSecond() noexcept {
	return (caerLogFileDescriptorsGetSecond());
}

void log(logLevel l, const char *subSystem, const char *format, ...) noexcept {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVA(static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)),
		subSystem, format, argumentList);
	va_end(argumentList);
}

void logVA(logLevel l, const char *subSystem, const char *format, va_list args) noexcept {
	caerLogVA(static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)),
		subSystem, format, args);
}

void logVAFull(int logFileDescriptor1, int logFileDescriptor2, uint8_t systemLogLevel, logLevel l,
	const char *subSystem, const char *format, va_list args) noexcept {
	caerLogVAFull(logFileDescriptor1, logFileDescriptor2, systemLogLevel,
		static_cast<enum caer_log_level>(static_cast<typename std::underlying_type<logLevel>::type>(l)), subSystem,
		format, args);
}

}
}
*/
#endif /* LIBCAER_HPP_ */

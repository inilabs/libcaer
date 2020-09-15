# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

# General build settings
IF (NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Possible build types: None Debug Release RelWithDebInfo MinSizeRel")
ENDIF()

IF (NOT CMAKE_INSTALL_PREFIX)
	IF (UNIX AND NOT APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake default install prefix, set to /usr on Unix/Linux")
	ELSEIF (APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "CMake default install prefix, set to /usr/local on macOS")
	ELSE()
		MESSAGE(FATAL_ERROR "CMAKE_INSTALL_PREFIX is not set")
	ENDIF()
ENDIF()

IF (NOT ENABLE_STATIC)
	SET(ENABLE_STATIC 0 CACHE BOOL "Build and install static library in addition to default shared library")
ENDIF()

IF (NOT ENABLE_SERIALDEV)
	SET(ENABLE_SERIALDEV 0 CACHE BOOL "Enable support for serial port devices using libserialport")
ENDIF()

IF (NOT ENABLE_OPENCV)
	SET(ENABLE_OPENCV 0 CACHE BOOL "Enable support for frame enhancements using OpenCV")
ENDIF()

IF (NOT UDEV_INSTALL)
	SET(UDEV_INSTALL 1 CACHE BOOL "Install udev rules on Linux")
ENDIF()

IF (NOT EXAMPLES_INSTALL)
	SET(EXAMPLES_INSTALL 0 CACHE BOOL "Build and install examples")
ENDIF()

# Project name and version
PROJECT(libcaer
	VERSION 3.3.8
	LANGUAGES C CXX)
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_C_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Custom CMake base module
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmakemod)
INSTALL(FILES cmakemod/caer-base.cmake DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer)

# Basic setup for cAER
INCLUDE(caer-base)
CAER_SETUP()

# Required: threads, math
SET(BASE_LIBS m)
SET(LIBCAER_PKGCONFIG_LIBS_PRIVATE "-lm")

SET(BASE_LIBS ${BASE_LIBS} ${SYSTEM_THREAD_LIBS})
SET(LIBCAER_PKGCONFIG_LIBS_PRIVATE "${LIBCAER_PKGCONFIG_LIBS_PRIVATE} ${SYSTEM_THREAD_LIBS}")

# Windows needs extra winsock library for portable endian functions.
IF (OS_WINDOWS)
	SET(BASE_LIBS ${BASE_LIBS} ws2_32)
ENDIF()

MESSAGE(STATUS "Base libraries: ${BASE_LIBS}")

# Search for external libraries with pkg-config.
INCLUDE(FindPkgConfig)

# Required: basic USB devices support
PKG_CHECK_MODULES(libusb REQUIRED IMPORTED_TARGET libusb-1.0>=1.0.17)
SET(LIBCAER_PKGCONFIG_REQUIRES_PRIVATE "libusb-1.0 >= 1.0.17")

# Optional: serial devices support
IF (ENABLE_SERIALDEV)
	# Require libserialport, minimum 0.1.1 version.
	PKG_CHECK_MODULES(libserialport REQUIRED IMPORTED_TARGET libserialport>=0.1.1)
	SET(LIBCAER_PKGCONFIG_REQUIRES_PRIVATE "${LIBCAER_PKGCONFIG_REQUIRES_PRIVATE}, libserialport >= 0.1.1")
ENDIF()

# Optional: OpenCV support for frame enhancement
IF (ENABLE_OPENCV)
	# OpenCV support.
	FIND_PACKAGE(OpenCV REQUIRED COMPONENTS core imgproc)
	IF (OpenCV_VERSION VERSION_LESS "3.1.0")
		MESSAGE(FATAL_ERROR "Cannot find OpenCV 3.1.0 or newer.")
	ENDIF()
ENDIF()

# Add main include dir for all targets
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include/)

# Subdirectories
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

# Generate pkg-config file
CONFIGURE_FILE(libcaer.pc.in libcaer.pc @ONLY)

INSTALL(FILES ${CMAKE_BINARY_DIR}/libcaer.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Export the cmake configuration for the install tree
INCLUDE(CMakePackageConfigHelpers)
WRITE_BASIC_CONFIG_VERSION_FILE(
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion)
SET(include_dirs ${CMAKE_INSTALL_INCLUDEDIR})
SET(export_destination ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
CONFIGURE_PACKAGE_CONFIG_FILE(
	${PROJECT_SOURCE_DIR}/cmakemod/${PROJECT_NAME}Config.cmake.in
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	INSTALL_DESTINATION ${export_destination}
	PATH_VARS include_dirs export_destination)
INSTALL(EXPORT libcaer-exports
	NAMESPACE libcaer::
	DESTINATION ${export_destination})
INSTALL(FILES
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	DESTINATION ${export_destination})
UNSET(include_dirs)
UNSET(export_destination)

# Install udev rules
IF (OS_LINUX AND UDEV_INSTALL)
	MESSAGE(STATUS "Installing global udev rules into /lib/udev/rules.d/")

	INSTALL(FILES
		docs/65-inivation.rules
		DESTINATION "/lib/udev/rules.d/")
ENDIF()

# Automatic documentation generation support
FIND_PACKAGE(Doxygen)

IF (DOXYGEN_FOUND)
	ADD_SUBDIRECTORY(docs)
ENDIF()

# Compile all examples
IF (EXAMPLES_INSTALL)
	ADD_SUBDIRECTORY(examples)
ENDIF()

# Support automatic RPM generation
SET(CPACK_PACKAGE_NAME ${PROJECT_NAME})
SET(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
SET(CPACK_PACKAGE_RELEASE 1)
SET(CPACK_PACKAGE_CONTACT "support@inivation.com")
SET(CPACK_PACKAGE_VENDOR "iniVation AG")
SET(CPACK_PACKAGE_DESCRIPTION "Minimal C library to access, configure and get/send AER data from sensors or to/from neuromorphic processors.")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${CPACK_PACKAGE_DESCRIPTION})
SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}.${CMAKE_SYSTEM_PROCESSOR}")
SET(CPACK_GENERATOR "RPM" "DEB")
SET(CPACK_RPM_PACKAGE_AUTOREQ 1)
#SET(CPACK_RPM_PACKAGE_REQUIRES "libusbx >= 1.0.17, libserialport >= 0.1.1, opencv >= 3.1.0")
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS 1)
#SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0 (>= 1.0.17), libserialport0 (>= 0.1.1), libopencv-contrib3.1 (>= 3.1.0)")

INCLUDE(CPack)

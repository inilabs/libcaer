# Set full RPATH
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

IF (OS_LINUX)
	# Raspberry Pi support available only on Linux.
	ADD_EXECUTABLE(davis_rpi davis_rpi.cpp)
	TARGET_LINK_LIBRARIES(davis_rpi PRIVATE caer)
	INSTALL(TARGETS davis_rpi DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

	ADD_EXECUTABLE(davis_rpi_benchmark davis_rpi_benchmark.cpp)
	TARGET_LINK_LIBRARIES(davis_rpi_benchmark PRIVATE caer)
	INSTALL(TARGETS davis_rpi_benchmark DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)
ENDIF()

IF (ENABLE_SERIALDEV)
	ADD_EXECUTABLE(edvs_simple edvs_simple.c)
	TARGET_LINK_LIBRARIES(edvs_simple PRIVATE caer)
	INSTALL(TARGETS edvs_simple DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

	ADD_EXECUTABLE(edvs_simple_cpp edvs_simple.cpp)
	TARGET_LINK_LIBRARIES(edvs_simple_cpp PRIVATE caer)
	INSTALL(TARGETS edvs_simple_cpp DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)
ENDIF()

IF (ENABLE_OPENCV)
	FIND_PACKAGE(OpenCV REQUIRED COMPONENTS core imgproc highgui)
	IF (OpenCV_VERSION VERSION_LESS "3.1.0")
		MESSAGE(FATAL_ERROR "Cannot find OpenCV 3.1.0 or newer.")
	ENDIF()

	ADD_EXECUTABLE(davis_cvgui davis_cvgui.cpp)
	TARGET_LINK_LIBRARIES(davis_cvgui PRIVATE caer ${OpenCV_LIBS})
	INSTALL(TARGETS davis_cvgui DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

	ADD_EXECUTABLE(davis_cvgui_filters davis_cvgui_filters.cpp)
	TARGET_LINK_LIBRARIES(davis_cvgui_filters PRIVATE caer ${OpenCV_LIBS})
	INSTALL(TARGETS davis_cvgui_filters DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

	ADD_EXECUTABLE(dvxplorer dvxplorer.cpp)
	TARGET_LINK_LIBRARIES(dvxplorer PRIVATE caer ${OpenCV_LIBS})
	INSTALL(TARGETS dvxplorer DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

	ADD_EXECUTABLE(samsung_evk samsung_evk.cpp)
	TARGET_LINK_LIBRARIES(samsung_evk PRIVATE caer ${OpenCV_LIBS})
	INSTALL(TARGETS samsung_evk DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

	ADD_EXECUTABLE(dvx_bw_test dvx_bw_test.cpp)
	TARGET_LINK_LIBRARIES(dvx_bw_test PRIVATE caer ${OpenCV_LIBS})
	INSTALL(TARGETS dvx_bw_test DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)
ENDIF()

ADD_EXECUTABLE(davis_enable_aer davis_enable_aer.cpp)
TARGET_LINK_LIBRARIES(davis_enable_aer PRIVATE caer)
INSTALL(TARGETS davis_enable_aer DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(davis_simple davis_simple.c)
TARGET_LINK_LIBRARIES(davis_simple PRIVATE caer)
INSTALL(TARGETS davis_simple DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(davis_simple_cpp davis_simple.cpp)
TARGET_LINK_LIBRARIES(davis_simple_cpp PRIVATE caer)
INSTALL(TARGETS davis_simple_cpp DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(davis_simple_2cam davis_simple_2cam.c)
TARGET_LINK_LIBRARIES(davis_simple_2cam PRIVATE caer)
INSTALL(TARGETS davis_simple_2cam DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(davis_text davis_text.cpp)
TARGET_LINK_LIBRARIES(davis_text PRIVATE caer)
INSTALL(TARGETS davis_text DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(device_discovery device_discovery.c)
TARGET_LINK_LIBRARIES(device_discovery PRIVATE caer)
INSTALL(TARGETS device_discovery DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(dvs128_simple dvs128_simple.c)
TARGET_LINK_LIBRARIES(dvs128_simple PRIVATE caer)
INSTALL(TARGETS dvs128_simple DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(dvs128_simple_cpp dvs128_simple.cpp)
TARGET_LINK_LIBRARIES(dvs128_simple_cpp PRIVATE caer)
INSTALL(TARGETS dvs128_simple_cpp DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

ADD_EXECUTABLE(dynapse_simple dynapse_simple.c)
TARGET_LINK_LIBRARIES(dynapse_simple PRIVATE caer)
INSTALL(TARGETS dynapse_simple DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/caer/examples)

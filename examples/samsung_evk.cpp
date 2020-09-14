#include <libcaercpp/devices/samsung_evk.hpp>

#include <atomic>
#include <csignal>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#if !defined(LIBCAER_HAVE_OPENCV) || LIBCAER_HAVE_OPENCV == 0
#	error "This example requires OpenCV support in libcaer to be enabled."
#endif

using namespace std;

static atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int signal) {
	// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
	if (signal == SIGTERM || signal == SIGINT) {
		globalShutdown.store(true);
	}
}

static void usbShutdownHandler(void *ptr) {
	(void) (ptr); // UNUSED.

	globalShutdown.store(true);
}

int main(void) {
// Install signal handler for global shutdown.
#if defined(_WIN32)
	if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#else
	struct sigaction shutdownAction;

	shutdownAction.sa_handler = &globalShutdownSignalHandler;
	shutdownAction.sa_flags   = 0;
	sigemptyset(&shutdownAction.sa_mask);
	sigaddset(&shutdownAction.sa_mask, SIGTERM);
	sigaddset(&shutdownAction.sa_mask, SIGINT);

	if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#endif

	// Open a DVS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
	auto handle = libcaer::devices::samsungEVK(1);

	// Let's take a look at the information we have on the device.
	auto info = handle.infoGet();

	printf("%s --- ID: %d, DVS X: %d, DVS Y: %d, Firmware: %d.\n", info.deviceString, info.deviceID, info.dvsSizeX,
		info.dvsSizeY, info.firmwareVersion);

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	handle.sendDefaultConfig();

	// Now let's get start getting some data from the device. We just loop in blocking mode,
	// no notification needed regarding new events. The shutdown notification, for example if
	// the device is disconnected, should be listened to.
	handle.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	handle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	cv::namedWindow("PLOT_EVENTS",
		cv::WindowFlags::WINDOW_AUTOSIZE | cv::WindowFlags::WINDOW_KEEPRATIO | cv::WindowFlags::WINDOW_GUI_EXPANDED);

	while (!globalShutdown.load(memory_order_relaxed)) {
		std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = handle.dataGet();
		if (packetContainer == nullptr) {
			continue; // Skip if nothing there.
		}

		printf("\nGot event container with %d packets (allocated).\n", packetContainer->size());

		for (auto &packet : *packetContainer) {
			if (packet == nullptr) {
				printf("Packet is empty (not present).\n");
				continue; // Skip if nothing there.
			}

			printf("Packet of type %d -> %d events, %d capacity.\n", packet->getEventType(), packet->getEventNumber(),
				packet->getEventCapacity());

			if (packet->getEventType() == POLARITY_EVENT) {
				std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
					= std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

				// Get full timestamp and addresses of first event.
				const libcaer::events::PolarityEvent &firstEvent = (*polarity)[0];

				int32_t ts = firstEvent.getTimestamp();
				uint16_t x = firstEvent.getX();
				uint16_t y = firstEvent.getY();
				bool pol   = firstEvent.getPolarity();

				printf("First polarity event - ts: %d, x: %d, y: %d, pol: %d.\n", ts, x, y, pol);

				cv::Mat cvEvents(info.dvsSizeY, info.dvsSizeX, CV_8UC3, cv::Vec3b{127, 127, 127});
				for (const auto &e : *polarity) {
					cvEvents.at<cv::Vec3b>(e.getY(), e.getX())
						= e.getPolarity() ? cv::Vec3b{255, 255, 255} : cv::Vec3b{0, 0, 0};
				}

				cv::imshow("PLOT_EVENTS", cvEvents);
				cv::waitKey(1);
			}
		}
	}

	handle.dataStop();

	// Close automatically done by destructor.

	cv::destroyWindow("PLOT_EVENTS");

	printf("Shutdown successful.\n");

	return (EXIT_SUCCESS);
}

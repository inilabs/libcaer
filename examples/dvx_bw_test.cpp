#include <libcaercpp/devices/dvxplorer.hpp>

#include <atomic>
#include <csignal>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#if !defined(LIBCAER_HAVE_OPENCV) || LIBCAER_HAVE_OPENCV == 0
#	error "This example requires OpenCV support in libcaer to be enabled."
#endif

using namespace std;

static atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int sig) {
	// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
	if ((sig == SIGTERM) || (sig == SIGINT)) {
		globalShutdown.store(true);
	}
}

static void usbShutdownHandler(void *ptr) {
	(void) (ptr); // UNUSED.

	globalShutdown.store(true);
}

int main() {
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

	if (sigaction(SIGTERM, &shutdownAction, nullptr) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, nullptr) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#endif

	// Open a DVS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
	auto handle = libcaer::devices::dvXplorer(1);

	// Let's take a look at the information we have on the device.
	auto info = handle.infoGet();

	printf("%s --- ID: %d, DVS X: %d, DVS Y: %d, Firmware: %d, Logic: %d.\n", info.deviceString, info.deviceID,
		info.dvsSizeX, info.dvsSizeY, info.firmwareVersion, info.logicVersion);

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	handle.sendDefaultConfig();

	handle.configSet(DVX_DVS_CHIP, DVX_DVS_CHIP_GLOBAL_HOLD_ENABLE, true);
	handle.configSet(DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, DVX_DVS_CHIP_BIAS_SIMPLE_VERY_HIGH);

	// Now let's get start getting some data from the device. We just loop in blocking mode,
	// no notification needed regarding new events. The shutdown notification, for example if
	// the device is disconnected, should be listened to.
	handle.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	handle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	cv::namedWindow("PLOT_EVENTS",
		cv::WindowFlags::WINDOW_AUTOSIZE | cv::WindowFlags::WINDOW_KEEPRATIO | cv::WindowFlags::WINDOW_GUI_EXPANDED);

	// Calculate event bandwidth statistics.
	int64_t smallTick         = 10; // Every 10us.
	int64_t nextSmallTick     = -1;
	uint64_t smallAccumulator = 0;
	float maxSmallMevts       = 0;

	int64_t bigTick         = 1000000; // Every 1s.
	int64_t nextBigTick     = -1;
	uint64_t bigAccumulator = 0;
	float maxBigMevts       = 0;

	while (!globalShutdown.load(memory_order_relaxed)) {
		std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = handle.dataGet();
		if (packetContainer == nullptr) {
			continue; // Skip if nothing there.
		}

		printf("\nGot event container with %d packets (allocated).\n", packetContainer->size());

		for (auto &packet : *packetContainer) {
			if (packet == nullptr) {
				continue; // Skip if nothing there.
			}

			printf("Packet of type %d -> %d events, %d capacity.\n", packet->getEventType(), packet->getEventNumber(),
				packet->getEventCapacity());

			if (packet->getEventType() == POLARITY_EVENT) {
				std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
					= std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

				if (nextSmallTick == -1) {
					// Initialize statistics.
					nextSmallTick = (*polarity)[0].getTimestamp64(*polarity) + smallTick;
					nextBigTick   = (*polarity)[0].getTimestamp64(*polarity) + bigTick;
				}

				cv::Mat cvEvents(info.dvsSizeY, info.dvsSizeX, CV_8UC3, cv::Vec3b{127, 127, 127});

				for (const auto &e : *polarity) {
					cvEvents.at<cv::Vec3b>(e.getY(), e.getX())
						= e.getPolarity() ? cv::Vec3b{255, 255, 255} : cv::Vec3b{0, 0, 0};

					// Update statistics.
					auto ts = e.getTimestamp64(*polarity);

					if (ts >= nextSmallTick) {
						// Calculate statistics on last sample.
						float mevts = static_cast<float>(smallAccumulator) / 10.0F;
						if (mevts > maxSmallMevts) {
							maxSmallMevts = mevts;
						}

						// Reset.
						smallAccumulator = 0;

						while (nextSmallTick <= ts) {
							nextSmallTick += smallTick;
						}
					}

					smallAccumulator++;

					if (ts >= nextBigTick) {
						// Calculate statistics on last sample.
						float mevts = static_cast<float>(bigAccumulator) / 1000000.0F;
						if (mevts > maxBigMevts) {
							maxBigMevts = mevts;
						}

						// Reset.
						bigAccumulator = 0;

						while (nextBigTick <= ts) {
							nextBigTick += bigTick;
						}
					}

					bigAccumulator++;
				}

				cv::imshow("PLOT_EVENTS", cvEvents);
				cv::waitKey(1);

				printf("Current max event rate: %f (momentary), %f (sustained) MEvt/s.\n", maxSmallMevts, maxBigMevts);
			}
		}
	}

	handle.dataStop();

	// Close automatically done by destructor.

	cv::destroyWindow("PLOT_EVENTS");

	printf("\nShutdown successful.\n");

	return (EXIT_SUCCESS);
}

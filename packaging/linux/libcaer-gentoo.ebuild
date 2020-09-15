# Copyright 2017-2019 iniVation AG
# Distributed under the terms of the GNU General Public License v2

EAPI=6

inherit eutils cmake-utils

DESCRIPTION="Minimal C library to access, configure and get data from neuromorphic sensors and processors."
HOMEPAGE="https://gitlab.com/inivation/dv/${PN}/"

SRC_URI="https://gitlab.com/inivation/dv/${PN}/-/archive/${PV}/${P}.tar.gz"

LICENSE="BSD-2"
SLOT="0"
KEYWORDS="amd64 arm x86"
IUSE="debug +serialdev +opencv static-libs"

RDEPEND=">=dev-libs/libusb-1.0.17
	serialdev? ( >=dev-libs/libserialport-0.1.1 )
	opencv? ( >=media-libs/opencv-3.1.0 )"

DEPEND="${RDEPEND}
	virtual/pkgconfig
	>=dev-util/cmake-3.10.0"

src_configure() {
	local mycmakeargs=(
		-DENABLE_SERIALDEV="$(usex serialdev 1 0)"
		-DENABLE_OPENCV="$(usex opencv 1 0)"
		-DENABLE_STATIC="$(usex static-libs 1 0)"
		-DUDEV_INSTALL=1
	)

	cmake-utils_src_configure
}

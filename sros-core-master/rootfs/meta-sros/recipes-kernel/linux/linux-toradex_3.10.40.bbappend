
FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append += "\
  file://0001-back-for-intel-realsense.patch \
  file://0002-Fix-GPIO6-usage-conflict.patch \
  file://0003-enable-smsc-network-adapter-and-set-rtc1-as-default-.patch \
  file://defconfig \
"


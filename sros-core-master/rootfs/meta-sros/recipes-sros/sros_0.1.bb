#
# This file was derived from the 'Hello World!' example recipe in the
# Yocto Project Development Manual.
#

DESCRIPTION = "Configure network, systemd etc"
SECTION = "sros"
LICENSE = "CLOSED"

# This tells bitbake where to find the files we're providing on the local filesystem
FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}-${PV}:"

DEPENDS = "bash"

RDEPENDS_${PN} += "bash"

# Use local tarball
SRC_URI = "\
  file://startup.sh \
  file://pre_startup_run.sh \
  file://pre_sros_run.sh \
  file://update_db.sh \
  file://network-monitor.sh \
  file://sros.service \
  file://network-monitor.service \
  file://10-enp3s0.network \
  file://11-enp1s0.network \
  file://20-eth0.network \
  file://db_config.csv \
  file://main.db3 \
  file://lib/libcxsparse.so.3.1.9 \
  file://lib/libiplplus.so.0.30.0 \
  file://lib/libprotobuf-lite.so.13.0.0 \
  file://lib/libopencv_core.so.2.4.8 \
  file://lib/libopencv_imgproc.so.2.4.8 \
  file://lib/libtcmalloc_minimal.so.4 \
  file://lib/libtcmalloc_and_profiler.so.4 \
"
# file://sshd_config

# Make sure our source directory (for the build) matches the directory structure in the tarball
S = "${WORKDIR}/sros-${PV}"

inherit autotools 


# do_compile() 内部不能有空格，否则会报错
do_compile() {
}

do_install() {

  install -d ${D}/sros/
  install -d ${D}/sros/db/
  install -m 0644 ${WORKDIR}/db_config.csv ${D}/sros/db/
  install -m 0644 ${WORKDIR}/main.db3 ${D}/sros/db/
  ln -s /sros/db/main.db3 ${D}/sros/db/config.db3

  install -d ${D}/sros/cfg/
  install -d ${D}/sros/map/
  install -d ${D}/sros/lib/
  install -d ${D}/sros/log/
  install -d ${D}/sros/monitor/

  install -d ${D}/sros/bin/
  install -m 0755 ${WORKDIR}/startup.sh ${D}/sros/bin/
  install -m 0755 ${WORKDIR}/pre_startup_run.sh ${D}/sros/bin/
  install -m 0755 ${WORKDIR}/pre_sros_run.sh ${D}/sros/bin/
  install -m 0755 ${WORKDIR}/update_db.sh ${D}/sros/bin/
  install -m 0755 ${WORKDIR}/network-monitor.sh ${D}/sros/bin/

  install -d ${D}/lib/systemd/system/
  install -m 0644 ${WORKDIR}/sros.service ${D}/lib/systemd/system/
  install -m 0644 ${WORKDIR}/network-monitor.service ${D}/lib/systemd/system/

  install -d ${D}/lib/systemd/network/
  install -m 0644 ${WORKDIR}/10-enp3s0.network ${D}/lib/systemd/network/
  install -m 0644 ${WORKDIR}/11-enp1s0.network ${D}/lib/systemd/network/
  install -m 0644 ${WORKDIR}/20-eth0.network ${D}/lib/systemd/network/

  install -d ${D}/usr/lib/

  install -m 0644 ${WORKDIR}/lib/libcxsparse.so.3.1.9 ${D}/usr/lib/
  ln -s /usr/lib/libcxsparse.so.3.19 ${D}/usr/lib/libcxsparse.so.3
  ln -s /usr/lib/libcxsparse.so.3.19 ${D}/usr/lib/libcxsparse.so

  install -m 0644 ${WORKDIR}/lib/libiplplus.so.0.30.0 ${D}/usr/lib/
  ln -s /usr/lib/libiplplus.so.0.30.0 ${D}/usr/lib/libiplplus.so.0.30 
  ln -s /usr/lib/libiplplus.so.0.30.0 ${D}/usr/lib/libiplplus.so 

  install -m 0644 ${WORKDIR}/lib/libprotobuf-lite.so.13.0.0 ${D}/usr/lib/
  ln -s /usr/lib/libprotobuf-lite.so.13.0.0 ${D}/usr/lib/libprotobuf-lite.so.13 
  ln -s /usr/lib/libprotobuf-lite.so.13.0.0 ${D}/usr/lib/libprotobuf-lite.so 

  install -m 0644 ${WORKDIR}/lib/libopencv_core.so.2.4.8 ${D}/usr/lib/
  ln -s /usr/lib/libopencv_core.so.2.4.8 ${D}/usr/lib/libopencv_core.so.2.4 
  ln -s /usr/lib/libopencv_core.so.2.4.8 ${D}/usr/lib/libopencv_core.so 

  install -m 0644 ${WORKDIR}/lib/libopencv_imgproc.so.2.4.8 ${D}/usr/lib/
  ln -s /usr/lib/libopencv_imgproc.so.2.4.8 ${D}/usr/lib/libopencv_imgproc.so.2.4 
  ln -s /usr/lib/libopencv_imgproc.so.2.4.8 ${D}/usr/lib/libopencv_imgproc.so

  install -m 0644 ${WORKDIR}/lib/libtcmalloc_minimal.so.4 ${D}/usr/lib/
  ln -s /usr/lib/libtcmalloc_minimal.so.4 ${D}/usr/lib/libtcmalloc_minimal.so

  install -m 0644 ${WORKDIR}/lib/libtcmalloc_and_profiler.so.4 ${D}/usr/lib/
  ln -s /usr/lib/libtcmalloc_and_profiler.so.4 ${D}/usr/lib/libtcmalloc_and_profiler.so

  #install -d ${D}/etc/ssh/
  #install -m 0644 ${WORKDIR}/sshd_config ${D}/etc/ssh/
}

FILES_${PN} += "\
  /sros/ \
  /sros/db/ \
  /sros/db/db_config.csv \
  /sros/db/main.db3 \
  /sros/cfg/ \
  /sros/map/ \
  /sros/log/ \
  /sros/monitor/ \
  /sros/bin/ \
  /sros/bin/startup.sh \
  /sros/bin/pre_startup_run.sh \
  /sros/bin/pre_sros_run.sh \
  /sros/bin/update_db.sh \
  /sros/bin/network-monitor.sh \
  /lib \
  /lib/systemd \
  /lib/systemd/system \
  /lib/systemd/system/sros.service \
  /lib/systemd/system/network-monitor.service \
  /lib/systemd/network \
  /lib/systemd/network/10-enp3s0.network \
  /lib/systemd/network/11-enp1s0.network \
  /lib/systemd/network/20-eth0.network \
  /usr/lib/libcxsparse.so.3.1.9 \
  /usr/lib/libiplplus.so.0.30.0 \
  /usr/lib/libprotobuf-lite.so.13.0.0 \
  /usr/lib/libopencv_core.so.2.4.8 \
  /usr/lib/libopencv_imgproc.so.2.4.8 \
  /usr/lib/libtcmalloc_minimal.so.4 \
  /usr/lib/libtcmalloc_and_profiler.so.4 \
  #/etc \
  #/etc/ssh \
  #/etc/ssh/sshd_config \
"

# The autotools configuration I am basing this on seems to have a problem with a race condition when parallel make is enabled
PARALLEL_MAKE = ""


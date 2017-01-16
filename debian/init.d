#!/bin/sh
# kFreeBSD do not accept scripts as interpreters, using #!/bin/sh and sourcing.
if [ true != "$INIT_D_SCRIPT_SOURCED" ] ; then
    set "$0" "$@"; INIT_D_SCRIPT_SOURCED=true . /lib/init/init-d-script
fi
### BEGIN INIT INFO
# Provides:          mcm-daemon
# Required-Start:    $syslog
# Required-Stop:     $syslog
# X-Stop-After:      sendsigs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mcm-daemon init script
# Description:       This script controls the daemon which manages
#		     the embedded microcontroller on some NAS devices.
#		     The daemon controls the system fan and some LEDs.
#		     It can be controlled via a network socket.
### END INIT INFO

# Author: Martin Mueller <mm@c-base.org>

DESC="MCU control daemon for fan and LEDs"
DAEMON=/sbin/mcm-daemon

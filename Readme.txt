This is a simple daemon for the WD-My Cloud Mirror gen2 and Ex2 ultra NAS system.

  It was developed on knowledge gained from reverse-engineering and spying 
  on the a D-Link dns320l system software and extended to the WD MyCloud 
  Gen2 devices (WD MyCloud Mirror Gen2 and WD MyCloud EX2 Ultra).
  Parts of this work are based on fan-daemon.py by Lorenzo Martignoni and 
  (c) 2013 Andreas Boehler, andreas _AT_ aboehler.at
Modified for WD  by
  (c) 2016 Carl Schiller, schreibcarl@gmail.com
  and
  (c) 2017 Martin Mueller, mm@c-base.org


How it works
============

  On bootup, mcm-daemon is executed and runs as a deamon. On startup, the 
  daemon can optionally read the RTC and set the system time to the RTC time.
  It also sends the DeviceReady command to the MCU so that the Power LED stops
  blinking.
  Afterwards, it goes to fan control loop.

  For normal operation a socket server on port 57367 is provided. You can
  connect with any telnet client to this port and work with the device.
  Just type "help" for a list of available commands.
  
Installation
============

  Either compile the software on your NAS by typing "make" or cross-compile
  it from your host computer using the supplied xcompile.sh script.
  You can also build a debian package with:
    dpkg-buildpackage -uc -us -b -t arm-linux-gnueabihf

  The mcm-daemon depends on the iniparser library from:
    http://github.com/ndevilla/iniparser

  Put the binary to /usr/bin and the config file to /etc. Adapt the parameters
  to your needs, leave to defaults if unsure.

  A systemd unit is provided which can be copied to /etc/systemd/system and
  enabled by running "systemctl enable mcm-daemon"

Disclaimer
==========

  This program is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program. If not, see <http://www.gnu.org/licenses/>.


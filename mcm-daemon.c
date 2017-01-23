/*
vim: ts=4 ai fdm=marker

	Simple system daemon for MyCloudMirror gen2; Ex2 Ultra

	(c) 2013 Andreas Boehler, andreas _AT_ aboehler.at
	(c) 2017 Martin Mueller, mm _AT_ sig21.net

	mod by C. Schiller, schreibcarl@gmail.com

	This code is based on a few other people's work and in parts shamelessly copied.
	The ThermalTable was provided by Lorenzo Martignoni and the fan control
	algorithm is based on his fan-daemon.py implementation.

	The MCU protocol was reverse engineered by strace() calls to up_send_daemon and
	up_read_daemon of the original firmware.

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

*/
#define _GNU_SOURCE

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <signal.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <dirent.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include <syslog.h>
#include <ctype.h>
#include <iniparser.h>

#include "mcm.h"
#include "mcm-daemon.h"

int ls;
int fd;
int ataPorts;
int fanRpm;
int fanSpeed;
tempState tsys;
tempState *tdisk;
DaemonConfig daemonCfg;

/** @file mcm-daemon.c
		@brief Implementation of a free system daemon replacement for
			the D-Link DNS-320L NAS
		@author Andreas Boehler, andreas _AT_ aboehler.at
		@version 1.0
		@date 2013/09/12
*/


int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd;
	//int len;
	char buf[100];
	char ch;

	//len = snprintf(buf, sizeof(buf), "%s/gpio%d/value", daemonCfg.gpioDir, gpio);

	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		syslog(LOG_ERR, "gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}

	close(fd);
	return 0;
}

void cleanup(int shut,int s,int howmany)
{
	int		 retval;

	/*
	 * Shutdown and close sock1 completely.
	 */
	if (shut)
	{
		retval = shutdown(s,howmany);
		if (retval == -1)
			syslog(LOG_ERR, "shutdown");
	}
	retval = close (s);
	if (retval)
		syslog(LOG_ERR, "close");
}

static void sighandler(int sig)
{
	syslog(LOG_DEBUG, "Signal Handler called\n");
	switch(sig)
	{
	case SIGINT:
		cleanup(0, ls, 1);
		exit(EXIT_SUCCESS);
		break;
	case SIGTERM:
		cleanup(0, ls, 1);
		if(daemonCfg.syncOnShutdown)
			HandleCommand("systohc", 7, NULL, 0);
		exit(EXIT_SUCCESS);
		break;
	}
}

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		syslog(LOG_ERR, "error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;		 // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;				// ignore break signal
	tty.c_lflag = 0;					// no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;					// no remapping, no delays
	tty.c_cc[VMIN]	= 0;				// read doesn't block
	tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		syslog(LOG_ERR, "error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		syslog(LOG_ERR, "error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]	= should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		syslog(LOG_ERR, "error %d setting term attributes", errno);
}

int CheckResponse(char *buf, char *cmd, int len)
{
	int i;
	//int tmp;
	int failure = 0;

	// Attention, 5 is hardcoded here and never checked!
	for(i=0;i<5;i++)
	{
		if(buf[i] != cmd[i])
		{
			syslog(LOG_ERR, "Char %i is %i but should be %i\n", i, buf[i], cmd[i]);
			failure = 1;
			break;
		}
	}
	if(failure)
	{
		for(i=0;i<len;i++)
		{
			syslog(LOG_DEBUG, "Buf/Cmd %i: %i %i\n", i, buf[i], cmd[i]);
		}
		return ERR_WRONG_ANSWER;
	}
/*	if(buf[len-1] != cmd[len-1])
	{
		syslog(LOG_ERR, "Last character does not match! Char %i is %i but should be %i\n", len-1, buf[len-1], cmd[len-1]);
		return ERR_WRONG_ANSWER;
	}
*/
	return SUCCESS;
}

void ClearSerialPort(int fd)
{
	char buf[100];
	struct pollfd fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;
	int n = 0;
	int pollrc;
	pollrc = poll(fds, 1, 0);
	if(pollrc > 0)
	{
		if(fds[0].revents & POLLIN)
		{
			syslog(LOG_DEBUG, "Clearing Serial Port...\n");
			do
			{
				n = read(fd, buf, sizeof(buf));
			} while(n == sizeof(buf));
		}
	}
}

int SendCommand(int fd, char *cmd, char *outArray)
{
	int nRetries = -1;
	int ret;
	do
	{
		ret = _SendCommand(fd, cmd, outArray);
		nRetries++;
		syslog(LOG_DEBUG, "Try number: %i\n", nRetries+1);
	} while((ret != SUCCESS) && (nRetries < daemonCfg.nRetries));

	return ret;
}

int _SendCommand(int fd, char *cmd, char *outArray)
{
	int n;
	int i;
	int j;
	ssize_t count;

	char buf[15];	// We need to keep the DateAndTime values here
					// Yes, we're sending byte by byte here - b/c the length of
					// commands and responses can vary!

	ClearSerialPort(fd);	// We clear the serial port in case
							// some old data from a previous request is still pending

	i=0;
	do
	{
		count = write(fd, &cmd[i], 1);
		i++;
		usleep(200);	// The MCU seems to need some time..
		if(count != 1)
		{
			syslog(LOG_ERR, "Error writing byte %i: %i, count: %ld\n", (i-1), cmd[i-1], (long int)count);
			return ERR_WRITE_ERROR;
		}
	} while(cmd[i-1] != CMD_STOP_MAGIC);

	i=0;
	do
	{
		n = read(fd, &buf[i], 1);
		i++;
		usleep(200);	// todo, replace with select and timeout
	} while((n == 1) && (buf[i-1] != CMD_STOP_MAGIC));


	if(buf[i-1] != CMD_STOP_MAGIC)
	{
		syslog(LOG_ERR, "Got no stop magic, but read %i bytes!\n", i);
		for(j=0;j<i;j++)
		{
			syslog(LOG_DEBUG, "Buf %i: %i\n", j, buf[j]);
		}
		return ERR_WRONG_ANSWER;
	}
	else
	{
		// If outArray is not NULL, an answer was requested
		if(outArray != NULL)
		{
			if(CheckResponse(buf, cmd, i) != SUCCESS)
			{
				return ERR_WRONG_ANSWER;
			}
			// Copy the answer to the outArray
			for(j=0; j<i; j++)
			{
				outArray[j] = buf[j];
			}
			usleep(50000);	// Give the µC some time to answer...

			// Wait for ACK from Serial
			i=0;
			do
			{
				n = read(fd, &buf[i], 1);
				i++;
			} while((n == 1) && (buf[i-1] != CMD_STOP_MAGIC));


			if(buf[i-1] != CMD_STOP_MAGIC)
			{
				syslog(LOG_ERR, "Got no stop magic!\n");
				for(j=0;j<i;j++)
				{
				 syslog(LOG_DEBUG, "Buf %i: %i\n", j, buf[j]);
				}

				return ERR_WRONG_ANSWER;
			}

			CheckResponse(buf, AckFromSerial, i);
			syslog(LOG_DEBUG, "Returning %i read bytes\n", n);
			return SUCCESS;
		}
		// Only wait for ACK if no response is expected
		else
		{
			return CheckResponse(buf, AckFromSerial, i);
		}
	}
}

static char* searchDisk(char *base, char *pref)
{
	DIR *dir;
	struct dirent *entry;
	char next[PATH_MAX];
	char *tmp;

	tmp = NULL;
	errno = 0;
	dir = opendir(base);
	if ( errno ) {
		syslog(LOG_ERR, "error(%d) opening directory: %s\n",errno, base);
		return NULL;
	}
	while ( (entry = readdir(dir)) != 0 ) {
		if ( strcmp("..",entry->d_name) == 0 || strcmp(".",entry->d_name) == 0 )
			continue;
		if ( strcmp(pref, "host") == 0 ) {
			if ( strncmp(pref,entry->d_name, strlen(pref)) == 0 ) {
				snprintf(next, PATH_MAX, "%s/%s", base, entry->d_name);
				tmp = searchDisk(next, "target");
			} else
				continue;
		} else if ( strcmp(pref, "target") == 0 ) {
			if ( strncmp(pref,entry->d_name, strlen(pref)) == 0 ) {
				snprintf(next, PATH_MAX, "%s/%s", base, entry->d_name);
				tmp = searchDisk(next, "");
			} else
				continue;
		} else if ( strlen(pref) == 0 ) {
			if ( strspn(entry->d_name, "0123456789:") == strlen(entry->d_name) ) {
				snprintf(next, PATH_MAX, "%s/%s", base, entry->d_name);
				tmp = searchDisk(next, "block");
			} else
				continue;
		} else if ( strcmp(pref, "block") == 0 ) {
			if ( strncmp(pref,entry->d_name, strlen(pref)) == 0 ) {
				snprintf(next, PATH_MAX, "%s/%s", base, entry->d_name);
				tmp = searchDisk(next, "sd");
			} else
				continue;
		} else if ( strcmp(pref, "sd") == 0 ) {
			if ( strncmp("sd", entry->d_name, 2) == 0 ) {
				asprintf(&tmp, "%s/%s", "/dev", entry->d_name);
				break;
			} else
				continue;
		}
	}
	closedir(dir);

	return tmp;
}

static int numAtaPorts(void) {
	int num;
	DIR *dir;
	struct dirent *entry;

	num = 0;
	errno = 0;
	dir = opendir(daemonCfg.ataPorts);
	if ( errno )
		syslog(LOG_ERR, "error %d enumerating ata ports\n",errno);
	else {
		while ( (entry = readdir(dir)) != 0 ) {
			if ( strncmp("ata",entry->d_name, 3) == 0 )
				num++;
		}
		closedir(dir);
	}
	syslog(LOG_DEBUG, "found %d ata ports\n",num);
	return num;
}

static char *getDisk(int disk) {
	int i;
	DIR *dir;
	struct dirent *entry;
	char *path, *tmp;

	tmp = NULL;
	if ( disk > ataPorts ) {
		//printf("disk %d > number of ports\n", disk);
		return NULL;
	}
	errno = 0;
	dir = opendir(daemonCfg.ataPorts);
	if ( errno )
		syslog(LOG_ERR, "error %d enumerating ata ports\n",errno);
	else {
		i = 0;
		while ( (entry = readdir(dir)) != 0 ) {
			if ( strncmp("ata",entry->d_name, 3) == 0 ) {
				i++;
				if ( i == disk ) {
					asprintf(&path, "%s/%s/device",daemonCfg.ataPorts, entry->d_name);
					tmp = searchDisk(path, "host");
					if (path)
						free(path);
				}
			}
		}
	}
	return tmp;
}

static int parseTemp(char *buf) {
	char *endptr = NULL;
	int temp = 0;
	int x;
	char *tok;

	x = 1;
	tok = strtok(buf, "\t\r\n ");
	while ( x < 10 && tok != NULL  ) {
		tok = strtok(NULL, "\t\r\n ");
		x++;
	}
	errno = 0;
	if (tok != NULL)
		temp = (int) strtol(tok, &endptr, 0);

	if ( errno == 0 && tok != endptr )
		return temp;

	return -1;
}

static int readHddTemp(int disk) {
	int fd[2];
	pid_t pid;
	char buf[2048], *dev;
	int tmp, temp;

	syslog(LOG_DEBUG, "query disk %d temperature\n", disk);
	dev = getDisk(disk);
	if ( dev == NULL )
		return 0;

	pipe(fd);
	pid = fork();
	if (pid < 0) {
		printf("fork() failed\n");
		return -1;
	}
	if ( pid == 0 ) {
		/* child to start smartctl */
		dup2(fd[1], 1);
		close(fd[0]);
		execlp("smartctl", "smartctl", "-A", dev, NULL);
	}
	dup2(fd[0],0);
	close(fd[1]);
	tmp = 0;
	temp = 0;
	while ( fgets(buf, 1024, stdin ) ) {
		if ( strncmp("190", buf, 3) == 0 ) {
			tmp = parseTemp(buf);
		}
		if ( strncmp("194", buf, 3) == 0 ) {
			tmp = parseTemp(buf);
		}
		if ( tmp > temp)
			temp = tmp;
	}
	//close(fd[0]);
	free(dev);

	return temp;
}

static int readFan()
{
	int rpm;
	char buf[10];
	if(SendCommand(fd, FanSpeedGetCmd, buf) > ERR_WRONG_ANSWER)
	{
		if ( buf[5] > 0 )
			rpm = (int) (300000 / buf[5]);
		else
			rpm = 0;

		syslog(LOG_DEBUG, "Read fan rpm: %i\n", rpm);
		return rpm;
	}

	syslog(LOG_DEBUG, "Error getting fan speed\n");
	return ERR_WRONG_ANSWER;
}

static int readSysTemp()
{
	char buf[15];
	int temp;

	if(SendCommand(fd, ThermalStatusGetCmd, buf) > ERR_WRONG_ANSWER)
	{
		temp = ThermalTable[(int)buf[5]];
		syslog(LOG_DEBUG, "Read system tempterature: %i °C\n", temp);
		return temp;
	}

	syslog(LOG_DEBUG, "Error reading system temperature\n");
	return ERR_WRONG_ANSWER;
}

static int setFanSpeed(int val)
{
	char buf[10];
	int i;

	for (i=0;i<=6;i++) {
		buf[i] = FanSpeedSetCmd[i];
	}
	buf[3] = (char)(val%256);
	buf[7] = 0;
	if(SendCommand(fd, buf, NULL) == SUCCESS)
		return 0;

	return 1;
}

static int DeviceReady(char *retMessage, int bufSize)
{
	if(SendCommand(fd, DeviceReadyCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int GetFanRpm(char *retMessage, int bufSize)
{
	int tmp, len;

	tmp = readFan();
	if ( tmp <= ERR_WRONG_ANSWER )
	{
		snprintf(retMessage, bufSize, "ERR %d\n", tmp);
		return 1;
	}

	snprintf(retMessage, bufSize, "%d", tmp);
	len = strlen(retMessage);
	if(bufSize > 1)
	{
		retMessage[len] = '\n';
		retMessage[len+1] = '\0';
	}
	return 0;
}

static int GetSysTemp(char *retMessage, int bufSize)
{
	int tmp, len;

	tmp = readSysTemp();
	if ( tmp <= ERR_WRONG_ANSWER )
	{
		snprintf(retMessage, bufSize, "ERR %d\n", tmp);
		return 1;
	}

	snprintf(retMessage, bufSize, "%d", tmp);
	len = strlen(retMessage);
	if(bufSize > 1)
	{
		retMessage[len] = '\n';
		retMessage[len+1] = '\0';
	}
	return 0;
}

static int quit(char *retMessage, int bufSize)
{
	strncpy(retMessage, "Bye\n", bufSize);
	return 2;
}

static int EnPwrRec(char *retMessage, int bufSize)
{
	if(SendCommand(fd, APREnableCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int DisPwrRec(char *retMessage, int bufSize)
{
	if(SendCommand(fd, APRDisableCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int GetPwrRec(char *retMessage, int bufSize)
{
	int len;
	char buf[15];


	if(SendCommand(fd, APRStatusCmd, buf) > ERR_WRONG_ANSWER)
	{
		snprintf(retMessage, bufSize, "%d", buf[5]);
		len = strlen(retMessage);
		if(bufSize > 1)
		{
			retMessage[len] = '\n';
			retMessage[len+1] = '\0';
		}
	}
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int EnWOL(char *retMessage, int bufSize)
{
	if(SendCommand(fd, WOLStatusEnableCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int DisWOL(char *retMessage, int bufSize)
{
	if(SendCommand(fd, WOLStatusDisableCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int GetWOL(char *retMessage, int bufSize)
{
	int len;
	char buf[15];

	if(SendCommand(fd, WOLStatusGetCmd, buf) > ERR_WRONG_ANSWER)
	{
		snprintf(retMessage, bufSize, "%d", buf[5]);
		len = strlen(retMessage);
		if(bufSize > 1)
		{
			retMessage[len] = '\n';
			retMessage[len+1] = '\0';
		}
	}
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedOff(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedOffCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedBlue(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedBlueOnCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedBlueBlink(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedBlueBlinkCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedBlueFade(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedBlueFadeCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedRed(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedRedOnCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedRedBlink(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedRedBlinkCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedOrange(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedOrangeOnCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int PwrLedOrangeBlink(char *retMessage, int bufSize)
{
	if(SendCommand(fd, PLedOrangeBlinkCmd, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int Shutdown(char *retMessage, int bufSize)
{
	strncpy(retMessage, "OK\n", bufSize);
	return 3;
}

static int ReadRtc(char *retMessage, int bufSize)
{
	int i;
	char buf[15];
	struct tm strTime;
	char timeStr[100];
	time_t rtcTime;

	if(SendCommand(fd, RDateAndTimeCmd, buf) > ERR_WRONG_ANSWER)
	{
		for(i=5;i<12;i++)
		{
			buf[i] = (buf[i] & 0x0f) + 10 * ((buf[i] & 0xf0) >> 4); // The other end is a µC (doh!)
		}
		strTime.tm_year = (100 + (int)buf[11]);
		strTime.tm_mon = buf[10]-1;
		strTime.tm_mday = buf[9];
		strTime.tm_hour = buf[7];
		strTime.tm_min = buf[6];
		strTime.tm_sec = buf[5];
		strTime.tm_isdst = -1;
		rtcTime = mktime(&strTime);
		strcpy(timeStr, ctime(&rtcTime));
		snprintf(retMessage, bufSize, "RTC: %s", timeStr);
	}
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int systohc(char *retMessage, int bufSize)
{
	int i;
	char cmdBuf[15];
	time_t sysTime;
	struct tm *strSetTime;

	// We do nothing, since it doesn't work yet
	return 0;

	// Copy the command to our buffer
	for(i=0;i<13;i++)
	{
		cmdBuf[i] = WDateAndTimeCmd[i];
	}
	sysTime = time(NULL);
	strSetTime = localtime(&sysTime);
	// Put the current local time into the command buffer
	cmdBuf[5] = (char)strSetTime->tm_sec;
	cmdBuf[6] = (char)strSetTime->tm_min;
	cmdBuf[7] = (char)strSetTime->tm_hour;
	cmdBuf[8] = (char)strSetTime->tm_wday;
	cmdBuf[9] = (char)strSetTime->tm_mday;
	cmdBuf[10] = (char)(strSetTime->tm_mon + 1);
	cmdBuf[11] = (char)(strSetTime->tm_year - 100);
	// And modify the values so that the MCU understands them...
	for(i=5;i<12;i++)
	{
		cmdBuf[i] = ((cmdBuf[i] / 10) << 4) + (cmdBuf[i] % 10);
	}
	if(SendCommand(fd, cmdBuf, NULL) == SUCCESS)
		strncpy(retMessage, "OK\n", bufSize);
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int hctosys(char *retMessage, int bufSize)
{
	int i;
	struct tm strTime;
	struct timeval setTime;
	time_t rtcTime;
	time_t sysTime;
	char timeStr[100];
	char buf[15];

	if(SendCommand(fd, RDateAndTimeCmd, buf) > ERR_WRONG_ANSWER)
	{
		for(i=5;i<12;i++)
		{
			buf[i] = (buf[i] & 0x0f) + 10 * ((buf[i] & 0xf0) >> 4); // The other end is a µC (doh!)
		}

		strTime.tm_year = (100 + (int)buf[11]);
		strTime.tm_mon = buf[10]-1;
		strTime.tm_mday = buf[9];
		strTime.tm_hour = buf[7];
		strTime.tm_min = buf[6];
		strTime.tm_sec = buf[5];
		strTime.tm_isdst = -1;
		rtcTime = mktime(&strTime);
		strcpy(timeStr, ctime(&rtcTime));
		// Retrieve system time
		sysTime = time(NULL);
		setTime.tv_sec = rtcTime;
		setTime.tv_usec = 0;
		// Set the time and print the difference on success
		if(settimeofday(&setTime, NULL) != 0)
			strncpy(retMessage, "ERR\n", bufSize);
		else
			snprintf(retMessage, bufSize, "RTC: %sSys: %sDiff: %.fs\n", timeStr, ctime(&sysTime), difftime(sysTime, rtcTime));
	}
	else
	{
		strncpy(retMessage, "ERR\n", bufSize);
		return 1;
	}
	return 0;
}

static int hddtemp(char *retMessage, int bufSize) {

	char *tmp;
	char buf[256];
	int i, c, pos;

	pos = 0;
	for (i=1;i<=ataPorts;i++) {
		tmp = getDisk(i);
		if ( tmp ) {
			c = snprintf(buf, 256, "slot %d %s temp: %d\n",i,tmp,readHddTemp(i));
			free(tmp);
			strncpy(&retMessage[pos], buf, bufSize - pos);
			pos += c;
		}
	}

	return 0;
}

static void updateHddTemp(void) {
	int i;

	for (i=0;i<ataPorts;i++) {
		tdisk[i].temp = readHddTemp(i + 1);
		if ( tdisk[i].temp == 0 )
			tdisk[i].tempOld = 0;
		if ( tdisk[i].tempOld == 0 )
			tdisk[i].tempOld = tdisk[i].temp;
	}
}

static void updateSysTemp(void) {

	tsys.temp = readSysTemp();
	if ( tsys.tempOld == 0 )
		tsys.tempOld = tsys.temp;
	if ( tsys.temp < 0 ) {
		tsys.temp = 0;
		tsys.tempOld = 0;
	}
}

static int checkTemps(void) {
	int temp = 0;
	int disks = 0;
	int i;


	for (i=0;i<ataPorts;i++) {
		if ( tdisk[i].temp > 0 ) {
			disks++;
			if (tdisk[i].temp < daemonCfg.tempDiskLow)
				temp--;
		}
	}
	if ( tsys.temp < daemonCfg.tempSysLow )
		temp--;

	if ( temp < (0 - disks) )
		temp = -2;
	else
		temp = 0;

	if ( temp == 0 ) {
		for (i=0;i<ataPorts;i++) {
			if (tdisk[i].tempOld < tdisk[i].temp && tdisk[i].temp > 0)
			temp++;
		}

		if ( tsys.tempOld < tsys.temp )
			temp++;

		if ( temp > 0 )
			temp = 1;
	}

	if ( temp == 0 ) {
		for (i=0;i<ataPorts;i++) {
			if (tdisk[i].tempOld > tdisk[i].temp && tdisk[i].temp > 0)
				temp--;
		}
		if ( tsys.tempOld > tsys.temp )
			temp--;

		if ( temp < 0 )
			temp = -1;
	}


	for (i=0;i<ataPorts;i++) {
		tdisk[i].tempOld = tdisk[i].temp;
	}
	tsys.tempOld = tsys.temp;

	return temp;
}

static int help(char *retMessage, int bufSize);

DaemonCommand cmdTable[] = {
		{ &DeviceReady, 	"DeviceReady",			"Tells the MCU, tht the device booted fully" },
		{ &GetFanRpm,		"GetFanRpm",			"Reads the fan speed from the MCU" },
		{ &GetSysTemp,		"GetSysTemperature",	"Reads the system temperatur from the MCU" },
		//{ &EnPwrRec,		"EnablePowerRecovery",	"Device boots after power failure" },
		//{ &DisPwrRec,		"DisablePowerRecovery",	"Device stays off after power failure" },
		//{ &GetPwrRec,		"GetPowerRecoveryState","Read status of power failure handling" },
		//{ &EnWOL,			"EnableWOL",			"enables wakup on magic packet" },
		//{ &DisWOL,		"DisableWOL",			"disables wakup on magic packet" },
		//{ &GetWOL,		"GetWOLState",			"read status of wakeonlan" },
		{ &PwrLedOff,		"PwrLedOff",			"switches the all power leds off" },
		{ &PwrLedBlue,		"PwrLedBlue",			"switches the power led to blue" },
		{ &PwrLedBlueBlink,	"PwrLedBlueBlink",		"blinks the power led blue" },
		{ &PwrLedBlueFade,	"PwrLedBlueFade",		"fades power led blue" },
		{ &PwrLedRed,		"PwrLedRed",			"switches the power led to red" },
		{ &PwrLedRedBlink,	"PwrLedRedBlink",		"blinks the power led red" },
		{ &PwrLedOrange,	"PwrLedOrange",			"switches the power led to orange" },
		{ &PwrLedOrangeBlink,"PwrLedOrangeBlink",	"blinks the power led orange" },
		{ &Shutdown,		"ShutdownDaemon",		"stops the mcm-daemon" },
		{ &hddtemp,			"hddTemp",				"print harddisk temperatures" },
		//{ &ReadRtc,		"ReadRtc",				"reads the realtime clock of the MCU" },
		//{ &systohc,		"systohc",				"sets realtime clock on the MCU to system time" },
		//{ &hctosys,		"hctosys",				"set system time from the MCU" },
		{ &help,			"help",					"shows this message" },
		{ &quit,			"quit",					"closes connection to daemon" },
	};

static int help(char *retMessage, int bufSize)
{
	int x,y,len,cmds;
	char fmt[20];

	cmds = sizeof(cmdTable)/sizeof(DaemonCommand);

	len=0;
	for (x=0; x<cmds; x++) {
		if (strlen(cmdTable[x].name)>len)
			len = strlen(cmdTable[x].name);
	}
	snprintf((char *)&fmt, 20, "%%%ds  %%s\n", len);
	y = 0;
	for (x=0; x<cmds; x++) {
		y += snprintf(&retMessage[y], bufSize - y, fmt, cmdTable[x].name, cmdTable[x].desc);
	}
	return 0;
}

int HandleCommand(char *message, int messageLen, char *retMessage, int bufSize) {
	uint8_t x;
	long num;
	char *endptr = NULL;
	int (*func)(char *retMessage, int bufSize);

	syslog(LOG_DEBUG, "Handling Command: %s\n", message);
	for (x=0;x<sizeof(cmdTable)/sizeof(DaemonCommand); x++) {
		if (strncmp(message,cmdTable[x].name, messageLen) == 0) {
			syslog(LOG_DEBUG, "%s\n", cmdTable[x].name);
			func = cmdTable[x].func;
			return func(retMessage, bufSize);
		}
	}
	errno = 0;
	num = strtol(message, &endptr, 0);
	if ( errno == 0 && message != endptr )
	{
		if ( num == 0 || (num >= 80 && num <= 255) ) {
			x = setFanSpeed((int)num);
			if ( x == 1 )
				strncpy(retMessage, "ERR set fan speed failed (0, 80 - 255)\n", bufSize);
			else
				strncpy(retMessage, "OK\n", bufSize);

			return x;
		} else {
			strncpy(retMessage, "ERR fan speed out of range\n", bufSize);
			return 0;
		}
	}
	strncpy(retMessage, "ERR Command not Understood\n", bufSize);
	return 0;
}

int main(int argc, char *argv[])
{
	char response[9000];
	int i;
	pid_t pid;
	pid_t sid;
	//int powerBtn;
	//int pressed;
	int opt;
	//int sleepCount;
	time_t sleepFan;
	time_t sleepHdd;
	time_t now;
	int adjust;
	int pollTimeMs;
	//int readRtcOnStartup = 0;
	//char buf[100];
	char *configPath = "/etc/mcm-daemon.ini";
	//char msgBuf[15];
	//int sysTemp;
	//int fanSpeed;
	//int fanRpm;
	struct sockaddr_in s_name;
	struct pollfd *fds = NULL;
	nfds_t nfds;
	int retval;
	int ret;
	int msgIdx;
	char message[500];
	dictionary *iniFile;
	socklen_t namelength;
	//pressed = 0;
	nfds = 1;
	opt = 1;
	//sleepCount = 0;
	pollTimeMs = 10; // Sleep 10ms for every loop
	fanSpeed = -1;
	//fanRpm = -1;

	daemonCfg.goDaemon = 1;
	daemonCfg.debug = 0;

	// Parse command line arguments
	while((i = getopt(argc, argv, "fc:d")) != -1)
	{
		switch(i)
		{
			case 'f':
				daemonCfg.goDaemon = 0;
				break;
			case 'd':
				daemonCfg.debug = 1;
				daemonCfg.goDaemon = 0;
				break;
			case 'c':
				configPath = optarg;
				break;
			case '?':
				if(optopt == 'c')
					fprintf(stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr,
									 "Unknown option character `\\x%x'.\n",
									 optopt);
				fprintf(stderr, "Usage: %s [-f] [-c configPath] [-d]\n", argv[0]);
				fprintf(stderr, "		 where\n");
				fprintf(stderr, "			 -f					don't detach\n");
				fprintf(stderr, "			 -c configPath		path to .ini\n");
				fprintf(stderr, "			 -d					debug (implies -f)\n");
				return EXIT_FAILURE;
		}

	}

	// Register some signal handlers
	signal(SIGTERM, sighandler);
	signal(SIGINT, sighandler);

	// Load our configuration file or use default values
	// if it doesn't exist!
	iniFile = iniparser_load(configPath);
	daemonCfg.portName			= iniparser_getstring(iniFile,	"Serial:Port", "/dev/ttyS1");
	daemonCfg.syncOnStartup		= iniparser_getint(iniFile,		"Daemon:SyncTimeOnStartup", 0);
	daemonCfg.fanPollTime		= iniparser_getint(iniFile,		"Fan:PollTime", 60);
	daemonCfg.tempSysLow		= iniparser_getint(iniFile,		"Fan:TempLow", 45);
	daemonCfg.tempSysHigh		= iniparser_getint(iniFile,		"Fan:TempHigh", 50);
	daemonCfg.speedMin			= iniparser_getint(iniFile,		"Fan:SpeedMin", 80);
	daemonCfg.speedMax			= iniparser_getint(iniFile,		"Fan:SpeedMax", 255);
	daemonCfg.hysteresis		= iniparser_getint(iniFile,		"Fan:Hysteresis", 2);
	daemonCfg.gpioPollTime		= iniparser_getint(iniFile,		"GPIO:PollTime", 1);
	daemonCfg.gpioDir			= iniparser_getstring(iniFile,	"GPIO:SysfsGpioDir", "/sys/class/gpio");
	daemonCfg.serverAddr		= iniparser_getstring(iniFile,	"Daemon:ServerAddr", "0.0.0.0");
	daemonCfg.serverPort		= iniparser_getint(iniFile,		"Daemon:ServerPort", 57367);
	daemonCfg.pollGpio			= iniparser_getint(iniFile,		"Daemon:PollGPIO", 1);
	daemonCfg.syncOnShutdown	= iniparser_getint(iniFile,		"Daemon:SyncTimeOnShutdown", 0);
	daemonCfg.nRetries			= iniparser_getint(iniFile,		"Serial:NumberOfRetries", 5);
	daemonCfg.ataPorts			= iniparser_getstring(iniFile,	"Disks:SysfsPorts", "/sys/class/ata_port");
	daemonCfg.hddPollTime		= iniparser_getint(iniFile,		"Disks:PollTime", 120);
	daemonCfg.tempDiskLow		= iniparser_getint(iniFile,		"Disks:TempLow", 38);
	daemonCfg.tempDiskHigh		= iniparser_getint(iniFile,		"Disks:TempHigh", 45);


	// Setup syslog
	if(daemonCfg.debug)
		setlogmask(LOG_UPTO(LOG_DEBUG));
	else
		setlogmask(LOG_UPTO(LOG_INFO));

	if(daemonCfg.goDaemon)
		openlog("mcm-daemon", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
	else
		openlog("mcm-daemon", LOG_CONS | LOG_PID | LOG_NDELAY | LOG_PERROR, LOG_LOCAL1);

	if(daemonCfg.goDaemon)
	{
		pid = fork();
		if(pid < 0)
		{
			syslog(LOG_ERR, "Forking failed.\n");
			return EXIT_FAILURE;
		}

		if(pid > 0)
		{
			return EXIT_SUCCESS;
		}
		// From here on we are the child process...
		umask(0);
		sid = setsid();
		if(sid < 0)
		{
			syslog(LOG_ERR, "Could not create process group\n");
			return EXIT_FAILURE;
		}

		if((chdir("/")) < 0)
		{
			 syslog(LOG_ERR, "Could not chdir(\"/\")\n");
			 return EXIT_FAILURE;
		}
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);

	}

	// Open our socket server
	if ((ls = socket (AF_INET, SOCK_STREAM, 0)) == -1){
		syslog(LOG_ERR, "socket");
		exit(EXIT_FAILURE);
	}

	if (setsockopt(ls,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof opt)<0){
		syslog(LOG_ERR, "setsockopt (SO_RESUSEADDR): %s\r\n",strerror(errno));
		exit(EXIT_FAILURE);
	}


	s_name.sin_family = AF_INET;
	s_name.sin_port = htons(daemonCfg.serverPort);
	s_name.sin_addr.s_addr = inet_addr(daemonCfg.serverAddr);

	syslog(LOG_DEBUG, "Bind name to ls. \n");
	retval = bind (ls,(struct sockaddr *)&s_name, sizeof s_name);
	if (retval)
	{
		syslog(LOG_ERR, "bind");
		cleanup(0, ls,1);
		exit(EXIT_FAILURE);
	}

	syslog(LOG_DEBUG, "Listen on ls for connections. \n");
	retval = listen (ls, 5);
	if (retval)
	{
		syslog(LOG_ERR, "listen");
		cleanup(0, ls,1);
		exit(EXIT_FAILURE);
	}
	syslog(LOG_INFO, "Server startup success on port %i\n", daemonCfg.serverPort);

	fds = (struct pollfd *)calloc(1,nfds*sizeof(struct pollfd));
	fds->fd = ls;
	fds->events = POLLIN | POLLPRI;

	fd = open (daemonCfg.portName, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		syslog(LOG_ERR, "error %d opening %s: %s", errno, daemonCfg.portName, strerror (errno));
		return 0;
	}

	set_interface_attribs (fd, B19200, 0);	// set speed to 19,200 bps, 8n1 (no parity) for WD MCM gen2; EX2u
	set_blocking (fd, 0);					// set no blocking


	// Send the DeviceReady command to the MCU
	if(SendCommand(fd, DeviceReadyCmd, NULL) == SUCCESS)
		syslog(LOG_INFO, "mcm-daemon startup complete, going to FanControl mode");
	else
	{
		syslog(LOG_ERR, "Error sending DeviceReady command, exit!\n");
		return EXIT_FAILURE;
	}

	if(daemonCfg.syncOnStartup)
	{
		syslog(LOG_INFO, "Setting system clock from RTC...\n");
		if(HandleCommand("hctosys", 7, NULL, 0) != 0)
			syslog(LOG_ERR, "Error setting system time from RTC!\n");
	}

	ataPorts = numAtaPorts();
	tdisk = malloc(ataPorts * sizeof(*tdisk));
	for(i=0;i<ataPorts;i++) {
		tdisk[i].temp    = 0;
		tdisk[i].tempOld = 0;
	}
	tsys.temp    = 0;
	tsys.tempOld = 0;

	fanSpeed = (daemonCfg.speedMax + daemonCfg.speedMin) / 2;
	setFanSpeed(fanSpeed);
	sleepFan = 0;
	sleepHdd = 0;

	// Go to endless loop and do the following:
	// Get the thermal status
	// Check temperature and adjust fan speeds
	// Wake every 1s to poll the power button GPIO
	// Wake every few ms to poll the sockets for connections
	// Sleep

	while(1)
	{
		now = time(NULL);

		if ((sleepFan + daemonCfg.fanPollTime) <= now) {
			sleepFan = now;
			updateSysTemp();
			fanRpm = readFan();
			syslog(LOG_DEBUG, "system temp: %d fan rpm: %d\n",tsys.temp, fanRpm);
		}

		if ((sleepHdd + daemonCfg.hddPollTime) <= now) {
			sleepHdd = now;
			updateHddTemp();
			for(i=0;i<ataPorts;i++) {
				if (tdisk[i].temp > 0)
					syslog(LOG_DEBUG, "hdd%d tempOld: %d, temp: %d\n",
						i, tdisk[i].tempOld, tdisk[i].temp);
			}
			syslog(LOG_DEBUG, "system tempOld: %d, temp: %d, fanSpeed: %d, fanRpm: %d\n",
				tsys.tempOld, tsys.temp, fanSpeed, fanRpm);
			adjust = checkTemps();
			if ( adjust == -2 )
				fanSpeed = 0;
			if ( adjust == -1 )
				fanSpeed *= 0.9;
			if ( adjust == 1 ) {
				if ( fanSpeed == 0 )
					fanSpeed = daemonCfg.speedMin;
				else
					fanSpeed *= 1.1;
			}

			if ( fanSpeed > daemonCfg.speedMax )
				fanSpeed = daemonCfg.speedMax;

			if ( fanSpeed > 0 && fanSpeed < daemonCfg.speedMin )
				fanSpeed = daemonCfg.speedMin;

			setFanSpeed(fanSpeed);

			if ( adjust != 0 )
				syslog(LOG_DEBUG, "adjusting fan speed: %d, adjust: %d\n", fanSpeed, adjust);
		}

		ret = poll(fds,nfds,pollTimeMs); // Time out after pollTimeMs
		if (ret == -1){
			syslog(LOG_ERR, "poll");
			exit(EXIT_FAILURE);
		}

		for (i=0;(i<nfds) && (ret);i++)
		{
			if (!(fds+i)->revents)
				continue;
			ret--;
			if (((fds+i)->fd == ls) && ((fds+i)->revents & POLLIN))
			{
				/*
				 * Accept connection from socket ls:
				 * accepted connection will be on socket (fds+nfds)->fd.
				 */
				syslog(LOG_DEBUG, "POLLIN on ls. Accepting connection\n");
				namelength = sizeof (s_name);
				fds = (struct pollfd *)realloc(fds,(nfds+1)*sizeof(struct pollfd));
				(fds+nfds)->fd	= accept (ls, (struct sockaddr *)&s_name, &namelength);
				if ((fds+nfds)->fd == -1)
				{
					syslog(LOG_ERR, "accept");
					cleanup(0, (fds+nfds)->fd, 1);
					fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
					continue;
				}
				(fds+nfds)->events = POLLIN | POLLRDNORM;
				nfds++;
				continue;
			}
			if ((fds+i)->revents & POLLNVAL)
			{
				syslog(LOG_DEBUG, "POLLNVAL on socket. Freeing resource\n");
				nfds--;
				memcpy(fds+i,fds+i+1,nfds-i);
				fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
				continue;
			}
			if ((fds+i)->revents & POLLHUP)
			{
				syslog(LOG_DEBUG, "POLLHUP => peer reset connection ...\n");
				cleanup(0,(fds+i)->fd,2);
				nfds--;
				memcpy(fds+i,fds+i+1,nfds-i);
				fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
				continue;
			}
			if ((fds+i)->revents & POLLERR){
				syslog(LOG_DEBUG, "POLLERR => peer reset connection ...\n");
				cleanup(0,(fds+i)->fd,2);
				nfds--;
				memcpy(fds+i,fds+i+1,nfds-i);
				fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
				continue;
			}
			if ((fds+i)->revents & POLLRDNORM)
			{
				retval = recv((fds+i)->fd, message, sizeof(message)-1, 0); // Don't forget the string terminator here!
				syslog(LOG_DEBUG, "-> (recv) retval = %d.\n",retval);	/* ped */
				msgIdx = retval;
				if (retval <=0)
				{
					if (retval == 0)
					{
						syslog(LOG_DEBUG, "recv()==0 => peer disconnected...\n");
						cleanup(1,(fds+i)->fd,2);
					}
					else
					{
						syslog(LOG_ERR, "receive");
						cleanup( 0, (fds+i)->fd,1);
					}
					nfds--;
					memcpy(fds+i,fds+i+1,nfds-i);
					fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
					continue;
				}
				while((retval > 0) && (message[msgIdx-2] != '\r') && ((msgIdx+1) < sizeof(message)))
				{
					retval = recv((fds+i)->fd, &message[msgIdx-2], sizeof(message) - retval - 1, 0);
					syslog(LOG_DEBUG, " \t -> (recv) retval = %d.\n", retval);
					if(retval > 0)
						msgIdx += retval - 2;
				}
				if(msgIdx > 1)
					if(message[msgIdx-1] == '\n')
					{
						if(message[msgIdx-2] == '\r')
							message[msgIdx-2] = '\0';
						else
							message[msgIdx-1] = '\0';
					}

				syslog(LOG_DEBUG, "Normal message :	%.*s\n",retval,message);
				msgIdx = HandleCommand(message, msgIdx, response, sizeof(response));
				retval = send((fds+i)->fd, response, strlen(response), 0);
				if((retval < 0) || (msgIdx > 1))
				{
					syslog(LOG_DEBUG, "send()==0 => peer disconnected...\n");
					cleanup(1,(fds+1)->fd, 2);
				}
				if(msgIdx == 3)
				{
					syslog(LOG_INFO, "Shutting down mcm-daemon...\n");
					return EXIT_SUCCESS;
				}
				continue;
			}
		}
	}
	closelog();
	iniparser_freedict(iniFile);
	return EXIT_SUCCESS;
}

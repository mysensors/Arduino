/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "log.h"
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#ifndef SERVICENAME
#define SERVICENAME "mysgw"
#endif

#define STR_HELPER(x) #x //!< Helper macro, STR_HELPER()
#define STR(x) STR_HELPER(x) //!< Helper macro, STR()

static const char *_log_level_colors[] = {
	"\x1b[1;5;91m", "\x1b[1;91m", "\x1b[91m", "\x1b[31m", "\x1b[33m", "\x1b[34m", "\x1b[32m", "\x1b[36m"
};
static const char *_log_level_names[] = {
	"EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG"
};
static uint8_t _log_quiet = 0;
static uint8_t _log_level = LOG_DEBUG;
static uint8_t _log_syslog = 0;

static uint8_t _log_pipe = 0;
static char *_log_pipe_file = NULL;
static int _log_pipe_fd = -1;

static FILE *_log_file_fp = NULL;

void logSetQuiet(uint8_t enable)
{
	_log_quiet = enable ? 1 : 0;
}

void logSetLevel(int level)
{
	if (level < LOG_EMERG || level > LOG_DEBUG) {
		return;
	}

	_log_level = level;
}

void logSetSyslog(int options, int facility)
{
	openlog(NULL, options, facility);
	_log_syslog = 1;
}

int logSetPipe(char *pipe_file)
{
	if (pipe_file == NULL) {
		return -1;
	}

	_log_pipe_file = strdup(pipe_file);
	if (_log_pipe_file == NULL) {
		return -1;
	}

	int ret = mkfifo(_log_pipe_file, 0666);
	if (ret == 0) {
		_log_pipe = 1;
	}

	logInfo("Using log pipe %s\n", _log_pipe_file);
	return ret;
}

int logSetFile(char *file)
{
	if (file == NULL) {
		return -1;
	}

	_log_file_fp = fopen(file, "a");
	if (_log_file_fp == NULL) {
		return errno;
	}

	logInfo("Using log file %s\n", file);
	return 0;
}

void logClose(void)
{
	if (_log_syslog) {
		closelog();
		_log_syslog = 0;
	}

	if (_log_pipe) {
		if (_log_pipe_fd > 0) {
			close(_log_pipe_fd);
		}
		/* remove the FIFO */
		unlink(_log_pipe_file);
		_log_pipe = 0;
	}
	if (_log_pipe_file != NULL) {
		free(_log_pipe_file);
		_log_pipe_file = NULL;
	}

	if (_log_file_fp != NULL) {
		fclose(_log_file_fp);
		_log_file_fp = NULL;
	}
}

void vlog(int level, const char *fmt, va_list args)
{
	if (_log_level < level) {
		return;
	}

	if (!_log_quiet || _log_file_fp != NULL) {
		/* Get current time */
		time_t t = time(NULL);
		struct tm *lt = localtime(&t);

		char date[16];
		date[strftime(date, sizeof(date), "%b %d %H:%M:%S", lt)] = '\0';

		if (_log_file_fp != NULL) {
			fprintf(_log_file_fp, "%s %s %-5s ", date, STR(SERVICENAME), _log_level_names[level]);
			vfprintf(_log_file_fp, fmt, args);
			fflush(_log_file_fp);
		}

		if (!_log_quiet) {
#ifdef LOG_DISABLE_COLOR
			(void)_log_level_colors;
			fprintf(stderr, "%s %s %-5s ", date, STR(SERVICENAME) _log_level_names[level]);
			vfprintf(stderr, fmt, args);
#else
			fprintf(stderr, "%s %s %s%-5s\x1b[0m ", date, STR(SERVICENAME), _log_level_colors[level],
			        _log_level_names[level]);
			vfprintf(stderr, fmt, args);
#endif
		}
	}

	if (_log_syslog) {
		vsyslog(level, fmt, args);
	}

	if (_log_pipe) {
		if (_log_pipe_fd < 0) {
			_log_pipe_fd = open(_log_pipe_file, O_WRONLY | O_NONBLOCK);
		}
		if (_log_pipe_fd > 0) {
			if (vdprintf(_log_pipe_fd, fmt, args) < 0) {
				close(_log_pipe_fd);
				_log_pipe_fd = -1;
			}
		}

	}
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logEmergency(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_EMERG, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logAlert(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_ALERT, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logCritical(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_CRIT, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logError(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_ERR, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logWarning(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_WARNING, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logNotice(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_NOTICE, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logInfo(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_INFO, fmt, args);
	va_end(args);
}

void
#ifdef __GNUC__
__attribute__((format(printf, 1, 2)))
#endif
logDebug(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vlog(LOG_DEBUG, fmt, args);
	va_end(args);
}

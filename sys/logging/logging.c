#include "logging.h"
#include "errors.h"
#include "console.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_MSG_SIZE 128U
#define MAX_FNAME_LINENUM_SIZE 128U
// Extra 10 for the small extra pieces in "%s - %s\r\n"
#define MAX_LOG_SIZE (MAX_MSG_SIZE + MAX_FNAME_LINENUM_SIZE + 10U)

#define UART_MUTEX_BLOCK_TIME portMAX_DELAY

static const char *LEVEL_STRINGS[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

static log_level_t logLevel;

void initLogger(void) {
  logLevel = LOG_DEFAULT_LEVEL;
}

void logSetLevel(log_level_t newLogLevel) { logLevel = newLogLevel; }

error_code_t logLog(log_level_t msgLevel, const char *file, uint32_t line, const char *s, ...) {
  if (msgLevel < logLevel) return ERR_CODE_LOG_MSG_SILENCED;

  if (file == NULL || s == NULL) return ERR_CODE_INVALID_ARG;

  int ret = 0;

  // Message
  char msgbuf[MAX_MSG_SIZE] = {0};
  va_list args;
  va_start(args, s);
  ret = vsnprintf(msgbuf, MAX_MSG_SIZE, s, args);
  va_end(args);
  if (ret < 0) return ERR_CODE_INVALID_ARG;
  if ((uint32_t)ret >= MAX_MSG_SIZE) return ERR_CODE_BUFF_TOO_SMALL;

  // File & line number
  char infobuf[MAX_FNAME_LINENUM_SIZE] = {0};
  ret = snprintf(infobuf, MAX_FNAME_LINENUM_SIZE, "%-5s -> %s:%u", LEVEL_STRINGS[msgLevel], file, line);
  if (ret < 0) return ERR_CODE_INVALID_ARG;
  if ((uint32_t)ret >= MAX_FNAME_LINENUM_SIZE) return ERR_CODE_BUFF_TOO_SMALL;

  // Prepare entire output
  char buf[MAX_LOG_SIZE] = {0};
  ret = snprintf(buf, MAX_LOG_SIZE, "%s - %s\r\n", infobuf, msgbuf);
  if (ret < 0) return ERR_CODE_INVALID_ARG;
  if ((uint32_t)ret >= MAX_LOG_SIZE) return ERR_CODE_BUFF_TOO_SMALL;

  printConsole((unsigned char *)buf);

  return ERR_CODE_SUCCESS;
}

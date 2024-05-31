/*
 * log.h
 *
 *  Created on: Feb 6, 2024
 *      Author: catsa
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <stdarg.h>

void Printf(const char *fmt, ...);
void dump_printf(const void *data, int size, const char* str);

#if defined(DEBUG)
#define LOG_ERR(fmt, ...) Printf("ERR: " fmt "\r\n", ##__VA_ARGS__);
#define LOG_WRN(fmt, ...) Printf("WRN: " fmt "\r\n", ##__VA_ARGS__);
#define LOG_DBG(fmt, ...) Printf("DBG: " fmt "\r\n", ##__VA_ARGS__);
#define LOG_INF(fmt, ...) Printf("INF: " fmt "\r\n", ##__VA_ARGS__);

#define LOG_HEX_DUMP(_data, _length, _str) dump_printf(_data, _length, _str)
#else
#define LOG_ERR(...) (void) 0
#define LOG_WRN(...) (void) 0
#define LOG_DBG(...) (void) 0
#define LOG_INF(...) (void) 0
#define LOG_HEX_DUMP(...) (void)0
#endif

#endif /* INC_LOG_H_ */

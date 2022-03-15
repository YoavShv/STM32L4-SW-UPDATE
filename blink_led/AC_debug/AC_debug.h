/**************************************************************************//**
 * @file    AC_debug.h
 * @brief   AC debug module definitions.
 * 
 * @copyright Rafael Advanced Defense Systems. All Rights Reserved (C).
 *****************************************************************************/
#ifndef _AC_DEBUG_H_
#define _AC_DEBUG_H_

/*****************************************************************************/
/*****************************************************************************/
/*                                 INCLUDES                                  */
/*****************************************************************************/
/*****************************************************************************/
#include <sys/stat.h>
#include <stdio.h>
#include "AC_defs.h"

/*****************************************************************************/
/*****************************************************************************/
/*                      INTERFACE FUNCTIONS DECLARATION                      */
/*****************************************************************************/
/*****************************************************************************/
/* redefinition functions for printf & scanf use*/
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

AC_status_t AC_DEBUG_uart_retarget_init(void);



#endif //#_AC_DEBUG_H_
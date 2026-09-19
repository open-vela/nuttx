/****************************************************************************
 * include/stdlib.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_STDLIB_H
#define __INCLUDE_STDLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The C standard specifies two constants, EXIT_SUCCESS and EXIT_FAILURE,
 * that may be passed to exit() to indicate successful or unsuccessful
 * termination, respectively.
 */

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

/* The NULL pointer should be defined in this file but is currently defined
 * in sys/types.h.
 */

/* Maximum value returned by rand().  Must be a minimum of 32767. */

#define RAND_MAX INT_MAX

/* Integer expression whose value is the maximum number of bytes in a
 * character specified by the current locale.
 */

#define MB_CUR_MAX 4

/* The environ variable, normally 'char **environ;' is not implemented as a
 * function call.  However, get_environ_ptr() can be used in its place.
 */

#ifdef CONFIG_DISABLE_ENVIRON
#  define environ (FAR char *[]){ NULL }
#else
#  define environ get_environ_ptr()
#endif

#if defined(CONFIG_FS_LARGEFILE)
#  define mkstemp64            mkstemp
#  define mkostemp64           mkostemp
#  define mkstemps64           mkstemps
#  define mkostemps64          mkostemps
#endif

#define strtof_l(s, e, l)      strtof(s, e)
#define strtod_l(s, e, l)      strtod(s, e)
#define strtold_l(s, e, l)     strtold(s, e)
#define strtoll_l(s, e, b, l)  strtoll(s, e, b)
#define strtoull_l(s, e, b, l) strtoull(s, e, b)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Structure type returned by the div() function.
 * NOTE: In C++ the toolchain C library (newlib) already provides div_t etc.,
 * and libstdc++'s <cstdlib> does include_next to the C stdlib, so defining
 * them here would conflict. Skip them in C++ mode (TFLM/GCC13 build fix).
 */

#if !defined(__cplusplus)

struct div_s
{
  int quot;     /* Quotient */
  int rem;      /* Remainder */
};

typedef struct div_s div_t;

/* Structure type returned by the ldiv() function. */

struct ldiv_s
{
  long quot;    /* Quotient */
  long rem;     /* Remainder */
};

typedef struct ldiv_s ldiv_t;

/* Structure type returned by the lldiv() function. */

struct lldiv_s
{
  long long int quot;    /* Quotient */
  long long int rem;     /* Remainder */
};

typedef struct lldiv_s lldiv_t;

#endif /* !defined(__cplusplus) */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Random number generation */

void      srand(unsigned int);
int       rand(void);
int       rand_r(FAR unsigned int *);
void      lcong48(FAR unsigned short int[7]);
FAR unsigned short int *seed48(FAR unsigned short int[3]);
void      srand48(long int);
#ifdef CONFIG_HAVE_LONG_LONG
long int  jrand48(FAR unsigned short int[3]);
long int  lrand48(void);
long int  mrand48(void);
long int  nrand48(FAR unsigned short int[3]);
#  ifdef CONFIG_HAVE_DOUBLE
double    drand48(void);
double    erand48(FAR unsigned short int[3]);
#  endif
#endif

#define   srandom(s) srand(s)
long      random(void);

void      arc4random_buf(FAR void *, size_t);
uint32_t  arc4random(void);

/* Environment variable support */

FAR char **get_environ_ptr(void);

FAR char *getenv(FAR const char *);

/* putenv is provided by the toolchain C library in C++ mode */
#if !defined(__cplusplus)
int       putenv(FAR const char *);
#endif

int       clearenv(void);

int       setenv(FAR const char *, FAR const char *, int);
int       unsetenv(FAR const char *);

/* Process exit functions */

void      exit(int) noreturn_function;

void      quick_exit(int) noreturn_function;

void      abort(void) noreturn_function;
int       atexit(CODE void (*)(void));
int       at_quick_exit(CODE void (*)(void));
int       on_exit(CODE void (*)(int, FAR void *), FAR void *);

/* _Exit() is a stdlib.h equivalent to the unistd.h _exit() function */

void      _Exit(int) noreturn_function;

/* System() command is not implemented in the NuttX libc because it is so
 * entangled with shell logic.  There is an experimental version at
 * apps/system/system.  system() is prototyped here, however, for
 * standards compatibility.
 */

#ifndef __KERNEL__
int       system(FAR const char *);
#endif

FAR char *realpath(FAR const char *, FAR char *);

/* String to binary conversions */

long      strtol(FAR const char *, FAR char **, int);
unsigned long strtoul(FAR const char *, FAR char **, int);
#ifdef CONFIG_HAVE_LONG_LONG
long long strtoll(FAR const char *, FAR char **, int);
unsigned long long strtoull(FAR const char *, FAR char **,
                            int);
#endif
float     strtof(FAR const char *, FAR char **);
#ifdef CONFIG_HAVE_DOUBLE
double    strtod(FAR const char *, FAR char **);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double strtold(FAR const char *, FAR char **);
#endif

int       atoi(FAR const char *);
long      atol(FAR const char *);
#ifdef CONFIG_HAVE_LONG_LONG
long long atoll(FAR const char *);
#endif
#ifdef CONFIG_HAVE_DOUBLE
double    atof(FAR const char *);
#endif

/* Binary to string conversions */

FAR char *itoa(int, FAR char *, int);

/* Wide character operations */

int       mblen(FAR const char *, size_t);
int       mbtowc(FAR wchar_t *, FAR const char *, size_t);
size_t    mbstowcs(FAR wchar_t *, FAR const char *, size_t);
int       wctomb(FAR char *, wchar_t);
size_t    wcstombs(FAR char *, FAR const wchar_t *, size_t);

/* Memory Management */

FAR void *malloc(size_t) malloc_like1(1);
FAR void *valloc(size_t) malloc_like1(1);
void      free(FAR void *);
FAR void *realloc(FAR void *, size_t) realloc_like(2);
FAR void *reallocarray(FAR void *, size_t, size_t) realloc_like2(2, 3);
FAR void *memalign(size_t, size_t) malloc_like1(2);
FAR void *zalloc(size_t) malloc_like1(1);
FAR void *aligned_alloc(size_t, size_t) malloc_like1(2);
FAR void *calloc(size_t, size_t) malloc_like2(1, 2);
int       posix_memalign(FAR void **, size_t, size_t);

/* Pseudo-Terminals */

#ifdef CONFIG_PSEUDOTERM
int       posix_openpt(int);
FAR char *ptsname(int);
int       ptsname_r(int, FAR char *, size_t);
int       unlockpt(int);

/* int grantpt(int fd); Not implemented */

#define grantpt(fd) (0)
#endif

/* Arithmetic */

int       abs(int);
long int  labs(long int);
#ifdef CONFIG_HAVE_LONG_LONG
long long int llabs(long long int);
#endif

#if !defined(__cplusplus)
div_t     div(int, int);
ldiv_t    ldiv(long, long);
#ifdef CONFIG_HAVE_LONG_LONG
lldiv_t   lldiv(long long, long long);
#endif
#endif /* !defined(__cplusplus) */

/* Temporary files */

FAR char *mktemp(FAR char *);
int       mkstemp(FAR char *);
FAR char *mkdtemp(FAR char *);

/* Sorting */

void      qsort(FAR void *, size_t, size_t,
                CODE int (*)(FAR const void *, FAR const void *));

/* Binary search */

FAR void  *bsearch(FAR const void *, FAR const void *, size_t,
                   size_t, CODE int (*)(FAR const void *,
                   FAR const void *));

/* Current program name manipulation */

FAR const char *getprogname(void);

/* Registers a destructor function to be called by exit() */

int __cxa_atexit(CODE void (*)(FAR void *), FAR void *,
                 FAR void *);

#if CONFIG_FORTIFY_SOURCE > 0
fortify_function(realpath) FAR char *realpath(FAR const char *path,
                                              FAR char *resolved)
{
  FAR char *ret = __real_realpath(path, resolved);
  if (ret != NULL && resolved != NULL)
    {
      size_t len = 1;
      FAR char *p;

      p = ret;
      while (*p++ != '\0')
        {
          len++;
        }

      fortify_assert(len <= fortify_size(resolved, 0));
    }

  return ret;
}
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDLIB_H */

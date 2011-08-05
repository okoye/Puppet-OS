/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * $Id: rtimer-arch.h,v 1.5 2010/02/18 17:21:44 dak664 Exp $
 */

#ifndef __RTIMER_ARCH_H__
#define __RTIMER_ARCH_H__

#include <avr/interrupt.h>

/* Nominal ARCH_SECOND is F_CPU/prescaler, e.g. 8000000/1024 = 7812 */
#ifndef RTIMER_ARCH_PRESCALER
#define RTIMER_ARCH_PRESCALER 1024UL
#endif
#define RTIMER_ARCH_SECOND (F_CPU/RTIMER_ARCH_PRESCALER)

/* Use TCNT1 if TCNT3 not available.
 * Setting RTIMER_ARCH_PRESCALER to 0 will leave timer1 alone.
 * Obviously this will disable rtimers.
 */

#ifdef TCNT3
#define rtimer_arch_now() (TCNT3)
#elif RTIMER_ARCH_PRESCALER
#define rtimer_arch_now() (TCNT1)
#else
#define rtimer_arch_now() (0)
#endif

#endif /* __RTIMER_ARCH_H__ */

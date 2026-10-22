/****************************************************************************
 *
 *   Copyright (C) 2021-2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file timer_config.cpp
 *
 * ESP32-P4 LEDC Timer and PWM Servo Channel Configuration
 */

#include <board_config.h>
#include <px4_arch/io_timer_hw_description.h>

/*
 * Timer allocation:
 * Timer 0 controls the first 4 channels (Main 1..4)
 * Timer 1 controls the next 4 channels (Aux 1..4)
 * Timer 2
 * Timer 3
 */
constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer0),
	initIOTimer(Timer::Timer1),
	initIOTimer(Timer::Timer2),
	initIOTimer(Timer::Timer3)
};

/*
 * Channel to physical GPIO Pin mapping
 * Pins are assigned to dedicated, conflict-free ESP32-P4 GPIOs
 */
constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer0, 0}, {0}),   /* Main 1 -> GPIO 0 */
	initIOTimerChannel(io_timers, {Timer::Timer0, 1}, {1}),   /* Main 2 -> GPIO 1 */
	initIOTimerChannel(io_timers, {Timer::Timer0, 2}, {6}),   /* Main 3 -> GPIO 6 */
	initIOTimerChannel(io_timers, {Timer::Timer0, 3}, {9}),   /* Main 4 -> GPIO 9 */
	initIOTimerChannel(io_timers, {Timer::Timer1, 0}, {10}),  /* Aux 1  -> GPIO 10 */
	initIOTimerChannel(io_timers, {Timer::Timer1, 1}, {11}),  /* Aux 2  -> GPIO 11 */
	initIOTimerChannel(io_timers, {Timer::Timer1, 2}, {14}),  /* Aux 3  -> GPIO 14 */
	initIOTimerChannel(io_timers, {Timer::Timer1, 3}, {15}),  /* Aux 4  -> GPIO 15 */
};

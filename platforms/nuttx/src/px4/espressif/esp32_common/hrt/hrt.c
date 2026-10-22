/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file drv_hrt.c
 *
 * High-resolution timer callouts and timekeeping.
 *
 * RP2040's internal 64 bit timer can be used for this purpose.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX RP2040 driver per se; rather, we
 * claim the timer and then drive it directly.
 */

#include <px4_platform_common/px4_config.h>
////////////////////////////////////
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
////////////////////////////////////
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <nuttx/queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/log.h>

/* ESP32-P4 64-bit Hardware Timer Support */
#include "esp_timer.h"
#include "esp_private/esp_clk.h"


/* Put them here so they are visible in ALL build modes: */
extern uint64_t esp_timer_impl_get_time(void);
extern void esp_timer_impl_early_init(void);


#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#if !defined(CONFIG_BUILD_FLAT)
#include <px4_platform_common/defines.h>
#include <px4_platform/board_ctrl.h>
#include <px4_platform_common/sem.h>

#define HRT_ENTRY_QUEUE_MAX_SIZE 3
static px4_sem_t g_wait_sem;
static struct hrt_call *next_hrt_entry[HRT_ENTRY_QUEUE_MAX_SIZE];
static int hrt_entry_queued = 0;
static bool suppress_entry_queue_error = false;
static bool hrt_entry_queue_error = false;

void hrt_usr_call(void *arg)
{
	if (hrt_entry_queued < HRT_ENTRY_QUEUE_MAX_SIZE) {
		next_hrt_entry[hrt_entry_queued++] = (struct hrt_call *)arg;

	} else {
		hrt_entry_queue_error = true;
	}

	px4_sem_post(&g_wait_sem);
}

#endif

#ifdef HRT_TIMER

/**
 * Minimum/maximum deadlines.
 */
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	4294951760LL

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint16_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint16_t			latency_actual;

/* latency histogram */
const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
const uint16_t latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t latency_counters[LATENCY_BUCKET_COUNT + 1];

/* timer-specific functions */
static void		hrt_tim_init(void);
static void		hrt_tim_isr(void *arg);
static void		hrt_latency_update(void);

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);

static esp_timer_handle_t tim;


////////////////////////////////////////////////////////////////////////////////
static struct wdog_s g_hrt_wdog;

static void hrt_wdog_cb(wdparm_t arg)
{
	(void)arg;
	hrt_call_invoke();
	hrt_call_reschedule();
}
////////////////////////////////////////////////////////////////////////////////



int hrt_ioctl(unsigned int cmd, unsigned long arg);

/**
 * Initialise the timer we are going to use.
 */
// static void
static void __attribute__((unused))

hrt_tim_init(void)
{
	irqstate_t flags = px4_enter_critical_section();

	const esp_timer_create_args_t timer_args = {
		.callback = &hrt_tim_isr,
		.arg = NULL,
		// .dispatch_method = ESP_TIMER_ISR,
		.dispatch_method = ESP_TIMER_TASK,
		// .dispatch_method = ESP_TIMER_MAX,
		.name = "px4_hrt",
		.skip_unhandled_events = true
	};

	esp_err_t err = esp_timer_create(&timer_args, &tim);

	if (err != ESP_OK) {
		PX4_ERR("ERROR: Failed to initialize ESP32P4 timer\n");
	}

	px4_leave_critical_section(flags);
}

/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static void __attribute__ ((section(".iram1")))
hrt_tim_isr(void *arg)
{
	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	/* and schedule the next interrupt */
	hrt_call_reschedule();
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime __attribute__ ((section(".iram1")))
hrt_absolute_time(void)
{
	// return (hrt_abstime)esp_timer_get_time();
	return (hrt_abstime)esp_timer_impl_get_time();
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
void __attribute__ ((section(".iram1")))
hrt_store_absolute_time(volatile hrt_abstime *t)
{
	irqstate_t flags = px4_enter_critical_section();
	*t = hrt_absolute_time();
	px4_leave_critical_section(flags);
}

/**
 * Initialise the high-resolution timing module.
 */
void
hrt_init(void)
{
	sq_init(&callout_queue);
	esp_timer_impl_early_init();
        // hrt_tim_init(); //////////////////////////



#if !defined(CONFIG_BUILD_FLAT)
	/* Create a semaphore for handling hrt driver callbacks */
	px4_sem_init(&g_wait_sem, 0, 0);
	/* this is a signalling semaphore */
	px4_sem_setprotocol(&g_wait_sem, SEM_PRIO_NONE);

	/* register ioctl callbacks */
	px4_register_boardct_ioctl(_HRTIOCBASE, hrt_ioctl);
#endif
}

/**
 * Call callout(arg) after interval has elapsed.
 */
void __attribute__ ((section(".iram1")))
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.
 */
void __attribute__ ((section(".iram1")))
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void __attribute__ ((section(".iram1")))
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void __attribute__ ((section(".iram1")))
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);
	px4_leave_critical_section(flags);
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool __attribute__ ((section(".iram1")))
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void __attribute__ ((section(".iram1")))
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;
	entry->period = 0;

	px4_leave_critical_section(flags);
}

static void __attribute__ ((section(".iram1")))
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				hrtinfo("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	hrtinfo("scheduled\n");
}

static void __attribute__ ((section(".iram1")))
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	while (true) {
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		hrtinfo("call pop\n");

		deadline = call->deadline;
		call->deadline = 0;

		if (call->callout) {
			hrtinfo("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
		}

		if (call->period != 0) {
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void __attribute__ ((section(".iram1")))
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	if (next != NULL) {
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			hrtinfo("pre-expired\n");
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			hrtinfo("due soon\n");
			deadline = next->deadline;
		}
	}

	hrtinfo("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	// /* Stop previous timer run and set exact absolute alarm time */
	// esp_timer_stop(tim);
	// esp_timer_start_once_at(tim, (uint64_t)deadline);



	////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* Use the NuttX tick (1 ms) as the alarm source for bring-up */
	hrt_abstime t = hrt_absolute_time();
	uint64_t rel = (deadline > t) ? (deadline - t) : 0;
	clock_t ticks = (clock_t)((rel + USEC_PER_TICK - 1) / USEC_PER_TICK);

	if (ticks < 1) {
		ticks = 1;
	}

	wd_cancel(&g_hrt_wdog);
	wd_start(&g_hrt_wdog, ticks, hrt_wdog_cb, 0);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
}

// static void
// hrt_latency_update(void)
static void __attribute__((unused))
hrt_latency_update(void)
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	latency_counters[index]++;
}

void __attribute__ ((section(".iram1")))
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void __attribute__ ((section(".iram1")))
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

#if !defined(CONFIG_BUILD_FLAT)

latency_info_t get_latency(uint16_t bucket_idx, uint16_t counter_idx)
{
	latency_info_t ret = {latency_buckets[bucket_idx], latency_counters[counter_idx]};
	return ret;
}

void reset_latency_counters(void)
{
	for (int i = 0; i <= get_latency_bucket_count(); i++) {
		latency_counters[i] = 0;
	}
}

int
hrt_ioctl(unsigned int cmd, unsigned long arg)
{
	hrt_boardctl_t *h = (hrt_boardctl_t *)arg;

	switch (cmd) {
	case HRT_WAITEVENT: {
			irqstate_t flags;
			px4_sem_wait(&g_wait_sem);
			flags = px4_enter_critical_section();

			if (hrt_entry_queued > 0) {
				*(struct hrt_call **)arg = next_hrt_entry[--hrt_entry_queued];
				next_hrt_entry[hrt_entry_queued] = NULL;

			} else {
				hrt_entry_queue_error = true;
			}

			px4_leave_critical_section(flags);

			if (hrt_entry_queue_error && !suppress_entry_queue_error) {
				PX4_ERR("HRT entry error, queue size now %d", hrt_entry_queued);
				suppress_entry_queue_error = true;
			}
		}
		break;

	case HRT_ABSOLUTE_TIME:
		*(hrt_abstime *)arg = hrt_absolute_time();
		break;

	case HRT_CALL_AFTER:
		hrt_call_after(h->entry, h->time, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CALL_AT:
		hrt_call_at(h->entry, h->time, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CALL_EVERY:
		hrt_call_every(h->entry, h->time, h->interval, (hrt_callout)hrt_usr_call, h->entry);
		break;

	case HRT_CANCEL:
		if (h && h->entry) {
			hrt_cancel(h->entry);

		} else {
			PX4_ERR("HRT_CANCEL called with NULL entry");
		}

		break;

	case HRT_GET_LATENCY: {
			latency_boardctl_t *latency = (latency_boardctl_t *)arg;
			latency->latency = get_latency(latency->bucket_idx, latency->counter_idx);
		}
		break;

	case HRT_RESET_LATENCY:
		reset_latency_counters();
		break;

	default:
		return -EINVAL;
	}

	return OK;
}
#endif

#endif /* HRT_TIMER */

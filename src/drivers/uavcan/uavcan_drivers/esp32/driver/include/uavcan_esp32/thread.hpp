#pragma once

#include <uavcan/uavcan.hpp>
#include <pthread.h>

namespace uavcan_esp32
{

class Mutex
{
	pthread_mutex_t mutex_;
public:
	Mutex() { pthread_mutex_init(&mutex_, nullptr); }
	~Mutex() { pthread_mutex_destroy(&mutex_); }
	void lock() { pthread_mutex_lock(&mutex_); }
	void unlock() { pthread_mutex_unlock(&mutex_); }
};

class MutexLocker
{
	Mutex& mutex_;
public:
	MutexLocker(Mutex& mutex) : mutex_(mutex) { mutex_.lock(); }
	~MutexLocker() { mutex_.unlock(); }
};

class BusEvent
{
public:
	typedef void (*SignalCallback)();

	BusEvent() : signal_cb_(nullptr) {}

	void registerSignalCallback(SignalCallback cb)
	{
		signal_cb_ = cb;
	}

	void signal()
	{
		if (signal_cb_) {
			signal_cb_();
		}
	}

private:
	SignalCallback signal_cb_;
};

} // namespace uavcan_esp32

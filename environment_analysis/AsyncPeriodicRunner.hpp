#ifndef ASYNC_TIMER_H_
#define ASYNC_TIMER_H_

#include <functional>
#include <chrono>
#include <future>

class AsyncPeriodicRunner
{
public:
	AsyncPeriodicRunner() :
		is_running(false)
	{

	}

	~AsyncPeriodicRunner()
	{
		if (is_running.load(std::memory_order_acquire))
			stop();
	}

	template <typename callable, typename... arguments>
	void start(unsigned interval, callable &&f, arguments&&... args)
	{
		using TimerFuncType = std::function<typename std::result_of<callable(arguments...)>::type()>;
		TimerFuncType task(std::bind(std::forward<callable>(f), std::forward<arguments>(args)...));

		if (is_running.load(std::memory_order_acquire))
			stop();

		is_running.store(true, std::memory_order_release);
		timer_thread = std::thread([this, interval, task]()
		{
			while (is_running.load(std::memory_order_acquire))
			{
				task();
				std::this_thread::sleep_for(std::chrono::milliseconds(interval));
			}
		});
	}

	void stop()
	{
		is_running.store(false, std::memory_order_release);
		if (timer_thread.joinable())
			timer_thread.join();
	}

	bool isRunning() const noexcept
	{
		return is_running.load(std::memory_order_acquire) && timer_thread.joinable();
	}

private:
	std::atomic<bool> is_running;
	std::thread timer_thread;
};

#endif
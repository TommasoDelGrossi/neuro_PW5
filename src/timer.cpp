#include "timer.hpp"
#include <chrono>

Timer::Timer() : start_time(-1), is_running(false) {}

long long int Timer::current_time_ms()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void Timer::start()
{
    if (!is_running)
    {
        start_time = current_time_ms();
        is_running = true;
    }
}

void Timer::reset()
{
    is_running = false;
    start_time = -1;
}

bool Timer::has_elapsed(int target_ms)
{
    if (!is_running)
    {
        start();            //se non è partito, lo faccio partire ora
    }

    if (current_time_ms() - start_time >= target_ms)
    {
        reset();            //resetto il timer per la prossima volta
        return true;
    }

    return false;
}

long long int Timer::get_elapsed_time_ms()
{
    if (!is_running)
    {
        return 0; // Se il timer non è in esecuzione, restituisci 0
    }
    return current_time_ms() - start_time;
}
#include "MThread.h"
#include "log.h"

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

void MThread::sleep(double seconds)
{
    DWORD ms = static_cast<DWORD>(seconds * 1000);
    timeBeginPeriod(1);
    ::Sleep(ms);
    timeEndPeriod(1);
}

void MThread::sleepMs(double msec)
{
    DWORD ms = static_cast<DWORD>(msec);
    timeBeginPeriod(1);
    ::Sleep(ms);
    timeEndPeriod(1);
}
#else
#include <time.h>
#include <errno.h>

// 辅助：纳秒加法
static void timespec_add_ns(struct timespec &ts, long ns)
{
    ts.tv_nsec += ns;
    while (ts.tv_nsec >= 1000000000L) {
        ts.tv_nsec -= 1000000000L;
        ts.tv_sec++;
    }
}

// sleep 绝对时间实现
static void sleep_until(const struct timespec &target)
{
    int ret;
    do {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
    } while (ret == EINTR);
}

void MThread::sleep(double seconds)
{
    struct timespec now, target;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long ns = static_cast<long>(seconds * 1e9);
    target = now;
    timespec_add_ns(target, ns);

    sleep_until(target);
}

void MThread::sleepMs(double msec)
{
    struct timespec now, target;
    clock_gettime(CLOCK_MONOTONIC, &now);

    long ns = static_cast<long>(msec * 1e6);
    target = now;
    timespec_add_ns(target, ns);

    sleep_until(target);
}
#endif

MThread::MThread()
{}

MThread::~MThread()
{
    if (!this->isStoped()) {
        this->stop();
    }

    if (this->th.joinable()) {
        this->th.join();
    }
}

void MThread::start()
{
    this->stopState = false;
    std::thread thr(&MThread::run, this);
    this->th = std::move(thr);
}

void MThread::stop()
{
    this->stopState = true;
    if (this->th.joinable()) {
        this->th.join();
    }
}

void MThread::join()
{
    this->th.join();
}

void MThread::detach()
{
    this->th.detach();
}

std::thread::id MThread::getId()
{
    return this->th.get_id();
}

bool MThread::isStoped()
{
    return this->stopState;
}

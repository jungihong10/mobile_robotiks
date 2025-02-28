#ifndef STEPPABLE_TASK
#define STEPPABLE_TASK

#include <coroutine>

struct SteppableTask {
    struct promise_type {
        SteppableTask get_return_object() {
            return SteppableTask(Handle::from_promise(*this));
        }

        auto initial_suspend() noexcept {
            return std::suspend_always();
        }

        auto final_suspend() noexcept {
            return std::suspend_always();
        }

        void return_void() {}

        //void unhandled_exception() {}
        [[noreturn]] void unhandled_exception() {
            throw;
        }
    };

    using Handle = std::coroutine_handle<promise_type>;

    SteppableTask() = delete;
    ~SteppableTask() {
        if (h) {
            h.destroy();
        }
    }

    SteppableTask(Handle h): h(h) {};

    SteppableTask(const SteppableTask &other) = delete;
    SteppableTask &operator=(const SteppableTask &other) = delete;

    SteppableTask(SteppableTask &&other) noexcept: h(other.h) {
        other.h = {};
    }

    SteppableTask &operator=(SteppableTask &&other) noexcept {
        if (this != &other) {
            if (h) {
                h.destroy();
            }
            h = other.h;
            other.h = {};
        }
        return *this;
    }

    void step() {
        if (!h.done()) {
            h.resume();
        }
    }

    Handle h;
};

#endif

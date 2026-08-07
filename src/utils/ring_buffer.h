#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// RingBuffer<T, N> — fixed-size circular buffer, not thread-safe on its own.
// Callers must hold a mutex if accessed from multiple FreeRTOS tasks.
//
// Used for: IMU sample history, PPG sample history, fall-event snapshots.
// ─────────────────────────────────────────────────────────────────────────────

template<typename T, size_t N>
class RingBuffer {
public:
    RingBuffer() : _head(0), _count(0) {
        memset(_buf, 0, sizeof(_buf));
    }

    // Push one sample. Overwrites oldest if full.
    void push(const T &sample) {
        _buf[_head] = sample;
        _head = (_head + 1) % N;
        if (_count < N) _count++;
    }

    // Read the n-th most recent sample (0 = most recent).
    // Returns false if fewer than n+1 samples are available.
    bool peek(size_t n, T &out) const {
        if (n >= _count) return false;
        size_t idx = (_head + N - 1 - n) % N;
        out = _buf[idx];
        return true;
    }

    size_t count() const { return _count; }
    bool   empty() const { return _count == 0; }
    bool   full()  const { return _count == N; }
    void   clear()       { _head = 0; _count = 0; }

    // Copy the last `len` samples into dst[0..len-1], oldest first.
    // Returns the number of samples actually copied (may be < len if buffer not full).
    size_t copyWindow(T *dst, size_t len) const {
        if (!dst || len == 0) return 0;
        size_t n = (len < _count) ? len : _count;
        for (size_t i = 0; i < n; i++) {
            // peek(n-1-i) gives oldest→newest order
            size_t idx = (_head + N - n + i) % N;
            dst[i] = _buf[idx];
        }
        return n;
    }

private:
    T      _buf[N];
    size_t _head;
    size_t _count;
};

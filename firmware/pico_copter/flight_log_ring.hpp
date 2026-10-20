#ifndef PICO_COPTER_FLIGHT_LOG_RING_HPP
#define PICO_COPTER_FLIGHT_LOG_RING_HPP

#include <stdint.h>

// Fixed-allocation sample ring. Stores complete rows; no allocation, heap,
// file writes or formatting in the 100 Hz flight-control path.
// Synchronization between Pico cores is provided by the caller.
template <uint32_t Rows, uint32_t Fields>
class FlightLogRing {
public:
    static_assert(Rows > 0 && Fields > 0, "A flight log needs rows and fields");

    void reset() {
        write_row_ = 0;
        used_rows_ = 0;
        total_rows_ = 0;
        frozen_ = false;
    }

    void freeze() { frozen_ = true; }

    bool append(const float (&values)[Fields]) {
        if (frozen_) return false;
        float* dest = samples_[write_row_];
        for (uint32_t i = 0; i < Fields; ++i) dest[i] = values[i];
        write_row_ = (write_row_ + 1) % Rows;
        if (used_rows_ < Rows) ++used_rows_;
        ++total_rows_;
        return true;
    }

    bool read_oldest_first(uint32_t logical_row, float (&values)[Fields]) const {
        if (logical_row >= used_rows_) return false;
        // Before wrap, the oldest row is 0; after wrap, it is write_row_.
        const uint32_t oldest = used_rows_ == Rows ? write_row_ : 0;
        const float* src = samples_[(oldest + logical_row) % Rows];
        for (uint32_t i = 0; i < Fields; ++i) values[i] = src[i];
        return true;
    }

    uint32_t count() const { return used_rows_; }
    uint32_t capacity() const { return Rows; }
    uint64_t total_rows() const { return total_rows_; }
    uint64_t first_sample_number() const { return total_rows_ - used_rows_; }
    bool frozen() const { return frozen_; }

private:
    float samples_[Rows][Fields] = {};
    uint32_t write_row_ = 0;
    uint32_t used_rows_ = 0;
    uint64_t total_rows_ = 0;
    bool frozen_ = false;
};

#endif

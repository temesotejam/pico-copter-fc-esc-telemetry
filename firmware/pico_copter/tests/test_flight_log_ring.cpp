#include "flight_log_ring.hpp"
#include <assert.h>
#include <stdio.h>

int main() {
    FlightLogRing<3, 2> ring;
    float row[2] = {};
    assert(ring.count() == 0 && ring.capacity() == 3);
    assert(!ring.read_oldest_first(0, row));
    assert(ring.first_sample_number() == 0);

    const float a[2] = {10, 11};
    const float b[2] = {20, 21};
    const float c[2] = {30, 31};
    const float d[2] = {40, 41};
    const float e[2] = {50, 51};
    assert(ring.append(a));
    assert(ring.append(b));
    assert(ring.count() == 2);
    assert(ring.read_oldest_first(0, row) && row[0] == 10 && row[1] == 11);
    assert(ring.read_oldest_first(1, row) && row[0] == 20);
    assert(!ring.read_oldest_first(2, row));

    assert(ring.append(c));
    assert(ring.append(d)); // Overwrites the oldest (10,11) as an entire sample.
    assert(ring.count() == 3 && ring.total_rows() == 4);
    assert(ring.first_sample_number() == 1);
    assert(ring.read_oldest_first(0, row) && row[0] == 20);
    assert(ring.read_oldest_first(1, row) && row[0] == 30);
    assert(ring.read_oldest_first(2, row) && row[0] == 40 && row[1] == 41);

    ring.freeze();
    assert(ring.frozen() && !ring.append(e));
    assert(ring.count() == 3 && ring.total_rows() == 4);
    assert(ring.read_oldest_first(0, row) && row[0] == 20);

    ring.reset();
    assert(!ring.frozen() && ring.count() == 0 && ring.total_rows() == 0);
    assert(!ring.read_oldest_first(0, row));
    assert(ring.append(e));
    assert(ring.read_oldest_first(0, row) && row[0] == 50 && row[1] == 51);
    assert(!ring.read_oldest_first(1, row));

    // Production CH5 recorder holds exactly 1000 whole 38-field samples.
    FlightLogRing<1000, 38>* real_type = nullptr;
    (void)real_type;
    static_assert(1000 * 38 * sizeof(float) == 152000, "Unexpected RAM size");
    puts("FlightLogRing partial/full/wrap/freeze/reset tests passed");
}

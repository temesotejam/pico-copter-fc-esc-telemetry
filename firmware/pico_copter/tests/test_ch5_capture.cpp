#include "log_capture_gate.hpp"
#include "flight_log_ring.hpp"
#include <cassert>
#include <cstdio>

int main() {
    using A=Ch5CaptureGate::Action;
    Ch5CaptureGate gate;
    FlightLogRing<1000, 38> log;
    float data[38]{};

    gate.on_arm();
    // An already-high CH5 before arming cannot start recording.
    assert(gate.update(true)==A::None);
    assert(gate.update(true)==A::None);
    assert(!gate.recording());
    assert(gate.update(false)==A::None);
    assert(gate.update(true)==A::Start);
    log.reset();

    // Record 1200 samples at 100 Hz: only the newest 1000 (10 seconds) remain.
    for (int i=0;i<1200;i++) {
        const A action = gate.update(true);
        assert(action==A::Capture);
        for(int j=0;j<38;j++) data[j]=float(i*100+j);
        assert(log.append(data));
    }
    assert(log.count()==1000 && log.first_sample_number()==200);
    float row[38]{};
    assert(log.read_oldest_first(0,row) && row[0]==20000 && row[37]==20037);
    assert(log.read_oldest_first(999,row) && row[0]==119900 && row[37]==119937);

    // CH5 OFF freezes the last samples, without clearing or overwriting.
    assert(gate.update(false)==A::Stop);
    log.freeze();
    assert(!gate.recording());
    for(int i=0;i<100;i++) assert(gate.update(false)==A::None);
    assert(log.count()==1000 && !log.append(data));
    assert(log.read_oldest_first(0,row) && row[0]==20000);

    // CH5 ON a second time deliberately starts a new capture.
    assert(gate.update(true)==A::Start);
    log.reset();
    assert(log.count()==0);
    for(int i=0;i<3;i++){
        assert(gate.update(true)==A::Capture);
        for(int j=0;j<38;j++) data[j]=float(i*100+j);
        assert(log.append(data));
    }
    assert(gate.update(false)==A::Stop);
    log.freeze();
    assert(log.count()==3);
    assert(log.read_oldest_first(0,row) && row[0]==0);
    assert(log.read_oldest_first(2,row) && row[0]==200);

    // Disarming closes recording; rearming never auto-starts even if CH5 high.
    gate.on_arm();
    assert(gate.update(true)==A::None);
    assert(gate.update(false)==A::None);
    assert(gate.update(true)==A::Start);
    gate.stop();
    log.freeze();
    assert(!gate.recording() && !log.append(data));
    std::puts("CH5 recording gate and ten-second ring tests passed");
}

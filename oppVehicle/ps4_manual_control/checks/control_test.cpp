#include "../main/control.h"
#include <cassert>
#include <cstdio>
using namespace hosim;
int main() {
    Control c;
    Input idle;
    Input drive;
    drive.enable = true;
    drive.driveY = -512;
    drive.x = 512;
    assert(c.motorUs == 1000 && c.steeringUs == 1600);
    c.packet(4900, 4900, idle);
    c.packet(5000, 5000, drive);
    assert(c.motorUs == 1000 && !c.ready); // Held at boot cannot start.
    c.packet(5010, 5010, idle);
    c.packet(5020, 5020, drive);
    assert(c.motorUs == 1180 && c.steeringUs == 1400);
    c.tick(5269);
    assert(c.motorUs == 1180);
    c.tick(5270);
    assert(c.motorUs == 1000 && c.steeringUs == 1600 && !c.ready);
    c.packet(5280, 5280, drive);
    assert(c.motorUs == 1000); // Timeout recovery requires release.
    c.packet(5290, 5290, idle);
    c.packet(5300, 5300, drive);
    c.packet(5600, 5600, drive);
    assert(c.motorUs == 1000); // Gap caught even without intermediate tick().
    c.packet(5610, 5610, idle);
    c.packet(5620, 5620, drive);
    Input released = drive;
    released.enable = false;
    c.packet(5630, 5630, released);
    assert(c.motorUs == 1000 && c.ready);
    c.packet(5640, 5640, drive);
    assert(c.motorUs == 1180); // Right stick still up: L1 toggle resumes throttle.
    c.packet(5650, 5650, idle);
    c.packet(5660, 5660, drive);
    drive.stop = true;
    c.packet(5670, 5670, drive);
    assert(c.motorUs == 1000 && !c.ready);
    drive.stop = false;
    c.packet(5680, 5680, drive);
    assert(c.motorUs == 1000);
    c.packet(5690, 5690, idle);
    c.packet(5700, 5700, drive);
    c.disconnect();
    c.packet(5710, 5710, drive);
    assert(c.motorUs == 1000);
    c.packet(5720, 5720, idle);
    uint32_t now = 5730;
    int previousSteer = 1800;
    for (int x = -512; x <= 512; ++x) {
        for (int y = -512; y <= 512; ++y) {
            Input in;
            in.enable = true;
            in.x = x;
            in.driveY = y;
            c.packet(now++, 6000, in);
            assert(c.motorUs >= 1000 && c.motorUs <= 1180);
            assert(c.steeringUs >= 1400 && c.steeringUs <= 1800);
            if (y >= -40) assert(c.motorUs == 1000);
            if (x >= -40 && x <= 40) assert(c.steeringUs == 1600);
            if (x == -512) assert(c.steeringUs == 1800);
            if (x == 512) assert(c.steeringUs == 1400);
        }
        assert(c.steeringUs <= previousSteer); // No reversals or jumps in direction.
        previousSteer = c.steeringUs;
    }
    Input steering;
    steering.enable = true;
    steering.x = -276; // Half of usable travel after deadzone.
    c.packet(now++, 6000, steering);
    assert(c.steeringUs == 1647 && c.motorUs == 1000);
    steering.x = 276;
    c.packet(now++, 6000, steering);
    assert(c.steeringUs == 1553 && c.motorUs == 1000);
    for (int x = 0; x <= 512; ++x) {
        steering.x = -x;
        c.packet(now++, 6000, steering);
        const int left = c.steeringUs;
        steering.x = x;
        c.packet(now++, 6000, steering);
        assert(left + c.steeringUs == 3200); // Mirrored input gives equal PWM offsets.
    }
    // Any valid stick position can re-arm when L1 is released.
    c.disconnect();
    Input down;
    down.driveY = 512;
    c.packet(now++, 6000, down);
    assert(c.ready && c.motorUs == 1000);
    down.enable = true;
    c.packet(now++, 6000, down);
    assert(c.ready && c.motorUs == 1000);
    c.packet(now++, 6000, idle);
    Input half = drive;
    half.driveY = -276;
    c.packet(now++, 6000, half);
    assert(c.motorUs == 1090);
    c.packet(now++, 6000, down);
    assert(c.motorUs == 1000);
    half.driveY = -513;
    c.packet(now++, 6000, half);
    assert(c.motorUs == 1000 && !c.ready);
    drive.x = 999;
    c.packet(now++, 6000, drive);
    assert(c.motorUs == 1000 && !c.ready);
    c.disconnect();
    c.packet(UINT32_MAX - 100, 6000, idle);
    drive.x = 0;
    c.packet(UINT32_MAX - 90, 6000, drive);
    c.tick(158);
    assert(c.motorUs == 1180);
    c.tick(159);
    assert(c.motorUs == 1000);
    puts("PASS: boot interlock, release/stop/disconnect, packet timeout/recovery, wraparound, 1,050,625 input combinations");
}

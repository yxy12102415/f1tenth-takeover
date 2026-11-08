#include "../main/control.h"
#include <cassert>
#include <cstdio>
using namespace hosim;
static void stopped(const Control& c) {
    assert(c.motorUs == kStop && c.steeringUs == kCenter);
}
static void moving(const Control& c) {
    assert(c.motorUs == kFixedThrottleUs && c.steeringUs == kCenter);
}
int main() {
    Control c;
    Input idle, drive;
    drive.enable = true; // No stick needed.
    stopped(c);
    c.packet(4900, 4900, idle);
    c.packet(5000, 5000, drive);
    stopped(c); assert(!c.ready);
    c.packet(5010, 5010, idle);
    c.packet(5020, 5020, drive); moving(c);
    c.tick(5269); moving(c);
    c.tick(5270); stopped(c); assert(!c.ready);
    c.packet(5280, 5280, drive); stopped(c);
    c.packet(5290, 5290, idle);
    c.packet(5300, 5300, drive); moving(c);
    c.packet(5600, 5600, drive); stopped(c); // Late packet cannot hide a gap.
    c.packet(5610, 5610, idle);
    c.packet(5620, 5620, drive); moving(c);
    c.packet(5630, 5630, idle); stopped(c); assert(c.ready);
    c.packet(5640, 5640, drive); moving(c);
    drive.stop = true;
    c.packet(5650, 5650, drive); stopped(c); assert(!c.ready);
    drive.stop = false;
    c.packet(5660, 5660, drive); stopped(c);
    c.packet(5670, 5670, idle);
    c.packet(5680, 5680, drive); moving(c);
    c.disconnect(); stopped(c);
    c.packet(5690, 5690, drive); stopped(c);
    c.packet(5700, 5700, idle);
    uint32_t now = 5710;
    for (int x = -512; x <= 512; ++x) {
        for (int y = -512; y <= 512; ++y) {
            drive.x = x; drive.driveY = y;
            c.packet(now++, 6000, drive); moving(c);
        }
    }
    c.packet(now++, 6000, idle); stopped(c);
    drive.x = 513;
    c.packet(now++, 6000, drive); stopped(c); assert(!c.ready);
    drive.x = 0;
    c.packet(now++, 6000, drive); stopped(c);
    c.packet(now++, 6000, idle);
    drive.driveY = -513;
    c.packet(now++, 6000, drive); stopped(c); assert(!c.ready);
    drive.driveY = 0;
    c.disconnect();
    c.packet(UINT32_MAX - 100, 6000, idle);
    c.packet(UINT32_MAX - 90, 6000, drive); moving(c);
    c.tick(158); moving(c);
    c.tick(159); stopped(c);
    puts("PASS fixed straight: L1 only, sticks ignored (1,050,625 combinations), release/stop/disconnect, boot/rearm, 250ms expiry including late packets, invalid input, timer wrap");
}

// DSR.cpp — Distance Sensor Reset algorithm.
//
// See the header for the high-level explanation. The math:
//
//   1. The IMU gives us the robot's heading θ. We use that to build cos θ
//      and sin θ once.
//   2. For each of the 4 sensors (any of which may be a nullptr), compute
//      its world-frame ray:
//        - sensor position = robot center + R(θ) * (mounting offset (X, Y))
//        - sensor direction = R(θ) * (mounting direction)
//      where R(θ) is WinLib's rotation matrix (heading is CW from +Y):
//        R(θ) = | -cos θ   sin θ |
//               |  sin θ   cos θ |
//   3. Use ray-vs-walls intersection to find which of the 4 walls the
//      sensor's ray hits — i.e. which wall the reading corresponds to.
//   4. Back out the robot's x or y from the sensor reading r:
//        if the ray hits an X wall at x = ±1800 mm:
//            x = wallCoord - offsetX - r * rayDirX
//        if it hits a Y wall:
//            y = wallCoord - offsetY - r * rayDirY
//   5. Filter the result by THREE checks before trusting it:
//        - perpendicularity (|rayDirX| or |rayDirY|, whichever axis we're
//          reading) must be >= MIN_PERPENDICULARITY (no near-grazing rays)
//        - back-out coordinate must be inside the field (sanity check —
//          out-of-field means the sensor is probably seeing a game piece)
//        - the standard valid-reading checks already done up top
//   6. Per axis, pick the sensor with the HIGHEST perpendicularity (i.e.
//      the ray most square-on to its wall) — that's the most reliable
//      reading. Update only the axes for which we have a winner. If
//      neither axis has a winner, this is a silent no-op (acceptable for
//      partial sensor setups at unfavorable headings).

#include "WinLib/chassis/DSR.hpp"
#include "WinLib/chassis/odom.hpp"
#include "WinLib/util.hpp"   // DegToRad
#include "WinLib/pose.hpp"
#include <cmath>
#include <limits>

using namespace WinLib;


DSR::DSR(pros::Distance* front, float frontOffsetX, float frontOffsetY,
         pros::Distance* back,  float backOffsetX,  float backOffsetY,
         pros::Distance* left,  float leftOffsetX,  float leftOffsetY,
         pros::Distance* right, float rightOffsetX, float rightOffsetY)
    : front(front), back(back), left(left), right(right),
      frontOffsetX(frontOffsetX), frontOffsetY(frontOffsetY),
      backOffsetX(backOffsetX),   backOffsetY(backOffsetY),
      leftOffsetX(leftOffsetX),   leftOffsetY(leftOffsetY),
      rightOffsetX(rightOffsetX), rightOffsetY(rightOffsetY) {}


namespace {

// One sensor's reading plus everything derived from it during reset().
struct Reading {
    float r;          // raw sensor distance reading (mm)
    float ox, oy;     // sensor face's world-frame offset from robot center
    float dx, dy;     // sensor ray's world-frame direction (unit vector)
    char  axis;       // 'X' if the ray hits east/west wall, 'Y' if north/south
    float coord;      // resulting x or y (depending on axis)
    float quality;    // perpendicularity to the hit wall, 0..1 (higher = better)
    bool  valid;      // false if we should skip this sensor
};

// Apply WinLib's rotation matrix to a robot-frame vector (vx, vy), with
// cos θ and sin θ already computed.
//
//   world = ( -cos θ * vx + sin θ * vy,
//             +sin θ * vx + cos θ * vy )
void rotateRobotToWorld(float vx, float vy, float c, float s,
                        float& wx, float& wy) {
    wx = -c * vx + s * vy;
    wy =  s * vx + c * vy;
}

// Given a ray that starts at world-frame point (sx, sy) and travels in
// direction (dx, dy), find which of the 4 walls it hits first.
// Returns true and sets axis ('X' or 'Y') + wallCoord (±FIELD_HALF) on hit.
// Returns false if the ray is parallel to all walls (shouldn't happen in
// practice — distance sensors always point in *some* direction).
bool intersectFirstWall(float sx, float sy, float dx, float dy,
                        float fieldHalf, char& axis, float& wallCoord) {
    float bestT = std::numeric_limits<float>::infinity();
    char  bestAxis = '?';
    float bestWall = 0;

    // East wall (x = +fieldHalf) — only reachable when dx > 0.
    if (dx > 1e-6f) {
        float t = (fieldHalf - sx) / dx;
        if (t > 0 && t < bestT) { bestT = t; bestAxis = 'X'; bestWall = fieldHalf; }
    }
    // West wall (x = -fieldHalf) — only reachable when dx < 0.
    else if (dx < -1e-6f) {
        float t = (-fieldHalf - sx) / dx;
        if (t > 0 && t < bestT) { bestT = t; bestAxis = 'X'; bestWall = -fieldHalf; }
    }
    // North wall (y = +fieldHalf) — only reachable when dy > 0.
    if (dy > 1e-6f) {
        float t = (fieldHalf - sy) / dy;
        if (t > 0 && t < bestT) { bestT = t; bestAxis = 'Y'; bestWall = fieldHalf; }
    }
    // South wall (y = -fieldHalf) — only reachable when dy < 0.
    else if (dy < -1e-6f) {
        float t = (-fieldHalf - sy) / dy;
        if (t > 0 && t < bestT) { bestT = t; bestAxis = 'Y'; bestWall = -fieldHalf; }
    }

    if (bestAxis == '?') return false;
    axis = bestAxis;
    wallCoord = bestWall;
    return true;
}

} // anonymous namespace


void DSR::reset() {
    // We don't bail if some sensors are missing — null sensors get skipped
    // and the algorithm uses whatever remains. Useful when the robot only
    // has 3 of the 4 sides instrumented (e.g. no back sensor yet).

    // Snapshot the current pose. We only ever overwrite (x, y); theta stays.
    Pose current = WinLib::getPose();
    const float c = std::cos(DegToRad(current.theta));
    const float s = std::sin(DegToRad(current.theta));

    // Sensor mounting positions and directions in robot frame.
    // Positions are arbitrary (X, Y) per sensor — see header for examples.
    // Directions are fixed by sensor name and never change.
    const float mountPos[4][2] = {
        {frontOffsetX, frontOffsetY},   // front
        {backOffsetX,  backOffsetY},    // back
        {leftOffsetX,  leftOffsetY},    // left
        {rightOffsetX, rightOffsetY}    // right
    };
    const float mountDir[4][2] = {
        {0,  +1},   // front points +y_robot
        {0,  -1},   // back  points -y_robot
        {+1, 0},    // left  points +x_robot
        {-1, 0}     // right points -x_robot
    };
    pros::Distance* sensors[4] = { front, back, left, right };

    // Build the 4 readings.
    Reading readings[4];
    for (int i = 0; i < 4; ++i) {
        // Skip any sensor that wasn't wired up.
        if (sensors[i] == nullptr) {
            readings[i].valid = false;
            continue;
        }
        float r = sensors[i]->get();
        readings[i].r = r;
        readings[i].valid = (r > 0 && r < NO_DETECTION);
        if (!readings[i].valid) continue;

        // Compute world-frame offset and direction for this sensor.
        rotateRobotToWorld(mountPos[i][0], mountPos[i][1], c, s,
                           readings[i].ox, readings[i].oy);
        rotateRobotToWorld(mountDir[i][0], mountDir[i][1], c, s,
                           readings[i].dx, readings[i].dy);

        // Sensor face's world position = robot pos + offset.
        float sx = current.x + readings[i].ox;
        float sy = current.y + readings[i].oy;

        char  axis;
        float wallCoord;
        if (!intersectFirstWall(sx, sy, readings[i].dx, readings[i].dy,
                                FIELD_HALF_MM, axis, wallCoord)) {
            readings[i].valid = false;
            continue;
        }
        readings[i].axis = axis;

        // Compute perpendicularity to the hit wall. For X-walls (east/west),
        // the wall normal is along x → perpendicularity = |rayDirX|. For
        // Y-walls (north/south), wall normal is along y → use |rayDirY|.
        float perp = (axis == 'X') ? std::fabs(readings[i].dx)
                                   : std::fabs(readings[i].dy);

        // Filter: if the ray is too grazing, drop this reading.
        if (perp < DSR::MIN_PERPENDICULARITY) {
            readings[i].valid = false;
            continue;
        }
        readings[i].quality = perp;

        // Back out the coordinate from the reading.
        //   If hits X wall:   x = wallCoord - offsetX - r * dirX
        //   If hits Y wall:   y = wallCoord - offsetY - r * dirY
        if (axis == 'X') {
            readings[i].coord = wallCoord - readings[i].ox - r * readings[i].dx;
        } else {
            readings[i].coord = wallCoord - readings[i].oy - r * readings[i].dy;
        }

        // Sanity: corrected coord must be inside the field. If we end up
        // back-calculating an x or y outside ±FIELD_HALF_MM, the sensor was
        // probably hitting a game piece or another robot, not a wall.
        if (std::fabs(readings[i].coord) > DSR::FIELD_HALF_MM) {
            readings[i].valid = false;
            continue;
        }
    }

    // Pick the HIGHEST-quality reading per axis. Quality = perpendicularity,
    // so the ray most square-on to its wall wins. This is the most reliable
    // reading regardless of how short or long it is.
    int bestX = -1, bestY = -1;
    for (int i = 0; i < 4; ++i) {
        if (!readings[i].valid) continue;
        if (readings[i].axis == 'X' && (bestX < 0 || readings[i].quality > readings[bestX].quality))
            bestX = i;
        if (readings[i].axis == 'Y' && (bestY < 0 || readings[i].quality > readings[bestY].quality))
            bestY = i;
    }

    // Update each axis only if we have evidence for it. If no sensor gives
    // usable X info, x stays at the current odom value (same for y). If
    // neither has a winner — e.g. a 2-sensor robot at a heading where both
    // sensors point the same way, or all sensors are grazing — the entire
    // reset is a silent no-op.
    float newX = (bestX >= 0) ? readings[bestX].coord : current.x;
    float newY = (bestY >= 0) ? readings[bestY].coord : current.y;

    WinLib::setPose(Pose(newX, newY, current.theta));
}

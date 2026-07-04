#pragma once
#include "pros/distance.hpp"

namespace WinLib {

/**
 * @brief DSR — Distance Sensor Reset.
 *
 * Uses 4 distance sensors (one on each side) and the IMU's heading to
 * compute the robot's true (x, y) position on the field and overwrite the
 * odometry pose. Heading is left untouched (the IMU is the source of truth).
 *
 * Why it exists:
 *   Odometry integrates wheel and IMU deltas every 10 ms. Each tick adds a
 *   tiny error (wheel slip, encoder noise, IMU drift) which compounds across
 *   a 90-second match — by the end of an auton route, the odom pose can be
 *   off by several inches. Distance sensors give an *absolute* fix: they
 *   measure distance to the wall, which doesn't drift. Calling DSR.reset()
 *   periodically wipes out the accumulated error.
 *
 * How it works (any heading, not just cardinal):
 *   1. Read the IMU's heading via WinLib::getPose().theta.
 *   2. For each of the 4 sensors, compute its position and ray direction
 *      in the world frame, given the robot's heading.
 *   3. Use ray–wall intersection math to find which of the 4 field walls
 *      the sensor's ray hits.
 *   4. Back out the robot's x (if the sensor hits an east/west wall) or y
 *      (if it hits a north/south wall) from the sensor's reading.
 *   5. Pick the two closest valid readings — one giving X info, one giving
 *      Y info — and overwrite the odom pose with the corrected (x, y).
 *
 * The "best perpendicularity wins" rule means we trust rays that hit the
 * wall most square-on. A ray hitting the wall at 89° (almost grazing) gives
 * a numerically valid but very fragile reading — small heading errors blow
 * up the answer. A ray hitting at 0° (perfectly perpendicular) gives the
 * most stable reading. Quality = |rayDirX| for X-wall hits, |rayDirY| for
 * Y-wall hits.
 *
 * Reset is **per-axis independent**. The algorithm always tries to update
 * x and y separately based on what readings are available:
 *   - Both axes have a usable reading → full reset.
 *   - Only one axis has a usable reading → that axis updates, other stays.
 *   - Neither has a usable reading → pose is left untouched (silent no-op).
 * This matters because a 3-sensor robot (or even a 2-sensor robot) can
 * still partially reset at most headings — the missing axis just waits
 * until another DSR call where it has data.
 *
 * Assumptions:
 *   - Field is the standard VEX 3600 mm × 3600 mm, origin at the **center**
 *     (walls at ±1800 mm in both axes).
 *   - Heading convention matches the odom module: 0° faces +Y (north),
 *     90° faces +X (east), increasing clockwise.
 *   - Each sensor sees the **wall** behind it, not a game piece or another
 *     robot. (Future work: sanity-check the corrected position against the
 *     current odom estimate, and reject if they disagree by more than some
 *     threshold.)
 *
 * Sensor mounting conventions (robot frame):
 *   - +y_robot = forward (the direction the robot's front faces)
 *   - +x_robot = the robot's "left" side, from the driver's POV
 *
 *   Each sensor gets its own **(X, Y) offset** in robot-frame mm, measured
 *   from the tracking center to the sensor's face. Real-world sensors aren't
 *   always centered on an axis — a "front" sensor mounted on the front-right
 *   corner is offset in both X (rightward → negative) and Y (forward →
 *   positive). The (X, Y) form captures that.
 *
 *   Examples:
 *     - front sensor dead-center on the bumper, 165 mm forward:
 *         frontOffsetX = 0,    frontOffsetY = 165
 *     - front sensor at the front-LEFT corner, 140 mm left, 165 mm forward:
 *         frontOffsetX = 140,  frontOffsetY = 165
 *     - left sensor at the middle of the LEFT side, 140 mm out:
 *         leftOffsetX  = 140,  leftOffsetY  = 0
 *     - right sensor at the middle of the RIGHT side, 140 mm out:
 *         rightOffsetX = -140, rightOffsetY = 0       (note: -X is "right")
 *
 *   The sensor **directions** are fixed by name (front faces +y_robot, back
 *   faces -y_robot, left faces +x_robot, right faces -x_robot) — only the
 *   mounting position varies between robots.
 *
 * @b Example
 * @code {.cpp}
 * pros::Distance frontDist(11), backDist(12), leftDist(13), rightDist(14);
 *
 * WinLib::DSR dsr(
 *     &frontDist, 0,    165,    // front: centered, 165 mm forward
 *     &backDist,  0,   -165,    // back:  centered, 165 mm rear
 *     &leftDist,  140,  0,      // left:  140 mm out on the left side
 *     &rightDist, -140, 0       // right: 140 mm out on the right side
 * );
 *
 * // ... later, anywhere in auton ...
 * dsr.reset();   // odom (x, y) snaps to the true position
 * @endcode
 */
class DSR {
public:
    DSR(pros::Distance* front, float frontOffsetX, float frontOffsetY,
        pros::Distance* back,  float backOffsetX,  float backOffsetY,
        pros::Distance* left,  float leftOffsetX,  float leftOffsetY,
        pros::Distance* right, float rightOffsetX, float rightOffsetY);

    /**
     * @brief Read all available sensors and overwrite the odom (x, y) with
     *        the corrected value. Heading is left untouched.
     *
     * Resets x and y INDEPENDENTLY based on what's available. A sensor is
     * dropped from consideration if:
     *   - it's a `nullptr` (so 3-sensor or 2-sensor robots work),
     *   - its reading is out of range (> NO_DETECTION) or non-positive,
     *   - its ray hits a wall at too grazing an angle (perpendicularity <
     *     MIN_PERPENDICULARITY), or
     *   - its back-out coordinate would put the robot outside the field.
     *
     * If no usable X-info reading exists, x is left at the current odom
     * value (same for y). If neither axis has a usable reading, the entire
     * reset is a silent no-op — useful detail when fewer than 4 sensors
     * are installed and the heading happens to point all of them at the
     * same axis or off-wall.
     */
    void reset();

    /**
     * @brief Left/right distance-sensor reading, in mm.
     *
     * Returns -1 if that side has no sensor (nullptr) or the reading is
     * invalid (non-positive, or >= NO_DETECTION i.e. nothing in range). Wall-
     * following motions (e.g. Chassis::moveByWall) read a side wall through
     * these instead of duplicating the sensor pointers — DSR already owns them.
     */
    float leftReading();
    float rightReading();

private:
    pros::Distance* front;
    pros::Distance* back;
    pros::Distance* left;
    pros::Distance* right;
    // Per-sensor (X, Y) offsets in robot frame, in mm. (0, 0) is the
    // tracking center; +y is forward, +x is the robot's left side.
    float frontOffsetX, frontOffsetY;
    float backOffsetX,  backOffsetY;
    float leftOffsetX,  leftOffsetY;
    float rightOffsetX, rightOffsetY;

    // Standard VEX field is 3600 mm × 3600 mm with origin at center.
    // Wall locations are at ±FIELD_HALF_MM in both X and Y.
    static constexpr float FIELD_HALF_MM = 1800.0f;
    // pros::Distance::get() returns this when nothing is detected.
    static constexpr float NO_DETECTION = 9999.0f;
    // Minimum perpendicularity required to trust a reading.
    //   1.0 = ray exactly perpendicular to the wall (ideal).
    //   0.0 = ray exactly parallel to the wall (never hits — useless).
    //   0.3 ≈ ray makes a 17° angle with the wall normal.
    // Below this threshold, the reading is too sensitive to small heading
    // errors and the answer can swing wildly. Skip the sensor in that case.
    static constexpr float MIN_PERPENDICULARITY = 0.3f;
};

} // namespace WinLib

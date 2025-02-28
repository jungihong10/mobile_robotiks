#include "obstacle_avoidance/vec2.h"
#include "../include/obstacle_avoidance/potential_field.h"
#include <vector>
#include <cmath>

PotentialField::PotentialField(double attGain, double repGain, double repulsionThreshold)
    : attractiveGain(attGain), repulsiveGain(repGain), repulsionThreshold(repulsionThreshold) {}

Vec2f PotentialField::calculateAttractiveForce(const Vec2f& goalPos, const Vec2f& robotPos) {
    Vec2f force = goalPos - robotPos;
    return  attractiveGain * force;
}

Vec2f PotentialField::calculateRepulsiveForce(const Vec2f& laserPoint, const Vec2f& robotPos) {
    Vec2f repulsiveForce(0, 0);

    Vec2f diff = robotPos - laserPoint;
    double distance = diff.norm();

    if (distance < repulsionThreshold) {
        Vec2f unitVector = diff.normalized();
        repulsiveForce += repulsiveGain * (1 / distance - 1 / repulsionThreshold) * (1 / (distance * distance)) * unitVector;
        }

    return repulsiveForce;
}

// Vec2f PotentialField::calculateTotalForce(const Vec2f& goalPos, const Vec2f& robotPos, const std::vector<Vec2f>& laserPoints) {
//     Vec2f attractiveForce = calculateAttractiveForce(goalPos, robotPos);
//     Vec2f repulsiveForce = calculateRepulsiveForce(laserPoints, robotPos);
//     return attractiveForce + repulsiveForce;
// }



// Vec2f PotentialField::calculateRepulsiveForce(const std::vector<Vec2f>& laserPoints, const Vec2f& robotPos) {
//     Vec2f repulsiveForce(0, 0);
//     for (const auto& point : laserPoints) {
//         Vec2f diff = robotPos - point;
//         double distance = diff.norm();

//         if (distance < repulsionThreshold) {
//             Vec2f unitVector = diff.normalized();
//             repulsiveForce += repulsiveGain * (1 / distance - 1 / repulsionThreshold) * (1 / (distance * distance)) * unitVector;
//         }
//     }
//     return repulsiveForce;
// }

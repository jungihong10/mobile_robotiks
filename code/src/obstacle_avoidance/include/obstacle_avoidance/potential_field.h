#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

#include <vector>
#include "vec2.h"

class PotentialField {
public:
    PotentialField(double attGain, double repGain, double repulsionThreshold);

    Vec2f calculateAttractiveForce(const Vec2f& goalPos, const Vec2f& robotPos);
    Vec2f calculateRepulsiveForce(const Vec2f& laserPoint, const Vec2f& robotPos);
    Vec2f calculateTotalForce(const Vec2f& goalPos, const Vec2f& robotPos, const std::vector<Vec2f>& laserPoints);

private:
    double attractiveGain;
    double repulsiveGain;
    double repulsionThreshold;
};

#endif // POTENTIAL_FIELD_H

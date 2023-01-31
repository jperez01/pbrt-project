#include "bounds.h"
#include <cmath>

Bounds3::Bounds3() {
    float minNum = std::numeric_limits<float>::lowest();
    float maxNum = std::numeric_limits<float>::max();

    pMin = glm::vec3(maxNum, maxNum, maxNum);
    pMax = glm::vec3(minNum, minNum, minNum);
}

Bounds3::Bounds3(const glm::vec3& p) : pMin(p), pMax(p) {}
Bounds3::Bounds3(const glm::vec3 &p1, const glm::vec3 &p2) {
    pMin = glm::vec3(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
    pMin = glm::vec3(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
}

glm::vec3 Bounds3::corner(int corner) const {
    return glm::vec3(
        (*this)[(corner & 1)].x,
        (*this)[(corner & 2) ? 1 : 0].y,
        (*this)[(corner & 4) ? 1 : 0].z);
}

glm::vec3 Bounds3::diagonal() const {
    return pMax - pMin;
}

glm::vec3 Bounds3::lerp(glm::vec3& t) const {
    return glm::vec3(
        pbrt::lerp(pMin.x, pMax.x, t.x),
        pbrt::lerp(pMin.y, pMax.y, t.y),
        pbrt::lerp(pMin.z, pMax.z, t.z)
    );
}

glm::vec3 Bounds3::offset(const glm::vec3& point) const {
    glm::vec3 o = point - pMin;
    if (pMax.x > pMin.y) o.x /= pMax.x - pMin.x;
    if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
    if (pMax.z > pMin.z) o.z /= pMax.z - pMin.z;

    return o;
}

void Bounds3::boundingSphere(glm::vec3& center, float& radius) const {
    glm::vec3 mix = pMin + pMax;
    center = 0.5f * mix;
    radius = Inside(center, *this) ? glm::distance(center, pMax) : 0;
}

float Bounds3::surfaceArea() const {
    glm::vec3 d = diagonal();
    return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
}

float Bounds3::volume() const {
    glm::vec3 d = diagonal();
    return d.x * d.y * d.z;
}

int Bounds3::maxExtent() const {
    glm::vec3 d = diagonal();
    if (d.x > d.y && d.x > d.z) return 0;
    else if (d.y > d.z) return 1;
    else return 2;
}

Bounds3 Union(const Bounds3 &bounds, const glm::vec3& point) {
    glm::vec3 min(
        std::min(bounds.pMin.x, point.x),
        std::min(bounds.pMin.y, point.y),
        std::min(bounds.pMin.z, point.z)
    );

    glm::vec3 max(
        std::max(bounds.pMax.x, point.x),
        std::max(bounds.pMax.y, point.y),
        std::max(bounds.pMax.z, point.z)
    );
    return Bounds3(min, max);
}

Bounds3 Union(const Bounds3 &bounds, const Bounds3& bounds2) {
    glm::vec3 min(
        std::min(bounds.pMin.x, bounds2.pMin.x),
        std::min(bounds.pMin.y, bounds2.pMin.y),
        std::min(bounds.pMin.z, bounds2.pMin.z)
    );

    glm::vec3 max(
        std::max(bounds.pMax.x, bounds2.pMax.x),
        std::max(bounds.pMax.y, bounds2.pMax.y),
        std::max(bounds.pMax.z, bounds2.pMax.z)
    );
    return Bounds3(min, max);
}

Bounds3 Intersect(const Bounds3 &bounds, const Bounds3& bounds2) {
    glm::vec3 min(
        std::max(bounds.pMin.x, bounds2.pMin.x),
        std::max(bounds.pMin.y, bounds2.pMin.y),
        std::max(bounds.pMin.z, bounds2.pMin.z)
    );

    glm::vec3 max(
        std::min(bounds.pMax.x, bounds2.pMax.x),
        std::min(bounds.pMax.y, bounds2.pMax.y),
        std::min(bounds.pMax.z, bounds2.pMax.z)
    );
    return Bounds3(min, max);
}

bool Overlaps(const Bounds3 &bounds, const Bounds3& bounds2) {
    bool x = (bounds.pMax.x >= bounds2.pMin.x) && (bounds.pMin.x <= bounds2.pMax.x);
    bool y = (bounds.pMax.y >= bounds2.pMin.y) && (bounds.pMin.y <= bounds2.pMax.y);
    bool z = (bounds.pMax.z >= bounds2.pMin.z) && (bounds.pMin.z <= bounds2.pMax.z);

    return (x && y && z);
}

bool Inside(const glm::vec3 &point, const Bounds3& bounds2) {
    bool x = (point.x >= bounds2.pMin.x) && (point.x <= bounds2.pMax.x);
    bool y = (point.y >= bounds2.pMin.y) && (point.y <= bounds2.pMax.y);
    bool z = (point.z >= bounds2.pMin.z) && (point.z <= bounds2.pMax.z);

    return (x && y && z);
}

bool InsideExclusive(const glm::vec3 &point, const Bounds3& bounds2) {
    bool x = (point.x >= bounds2.pMin.x) && (point.x < bounds2.pMax.x);
    bool y = (point.y >= bounds2.pMin.y) && (point.y < bounds2.pMax.y);
    bool z = (point.z >= bounds2.pMin.z) && (point.z < bounds2.pMax.z);

    return (x && y && z);
}

Bounds3 Expand(const Bounds3 &bounds, float delta) {
    return Bounds3(
        bounds.pMin - glm::vec3(delta, delta, delta),
        bounds.pMax + glm::vec3(delta, delta, delta));
}
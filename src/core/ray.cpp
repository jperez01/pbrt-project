#include "ray.h"
#include "geometry.h"

Ray::Ray() : tMax(Infinity), time(0.f) {}
Ray::Ray(const glm::vec3& org, const glm::vec3& dir, 
    float tMax = Infinity, float time = 0.f) :
    o(org), dir(dir), tMax(tMax), time(time) {}

bool Ray::hasNans() {
    return pbrt::hasNan(o) || pbrt::hasNan(dir) || std::isnan(tMax);
}

glm::vec3 Ray::operator()(float t) {
    return o + dir * t;
}

std::ostream& operator<<(std::ostream& os, const Ray r) {
    os << "Origin: " << r.o << "\n"
        << "Direction: " << r.dir << "\n"
        << "tMax: " << r.tMax << "\n"
        << "Time: " << r.time << "\n";
}

RayDifferential::RayDifferential() {
    hasDifferentials = false;
}

RayDifferential::RayDifferential(const glm::vec3& o, const glm::vec3& d,
    float tMax = Infinity, float time = 0.f) : Ray(o, d, tMax, time) {
    hasDifferentials = false;
}

RayDifferential::RayDifferential(const Ray& ray) : Ray(ray) {
    hasDifferentials = false;
}

bool RayDifferential::hasNans() {
    return Ray::hasNans() ||
        (hasDifferentials && (
            pbrt::hasNan(rxOrigin) || pbrt::hasNan(ryOrigin) ||
            pbrt::hasNan(rxDirection) || pbrt::hasNan(ryDirection)
        ));
}
void RayDifferential::scaleDifferentials(float s) {
    rxOrigin = o + (rxOrigin - o) * s;
    ryOrigin = o + (ryOrigin - o) * s;

    rxDirection = dir + (rxDirection - dir) * s;
    ryDirection = dir + (ryDirection - dir) * s;
}
#pragma once
#include "geometry.h"

class Bounds2 {
    public:
        Bounds2();
        glm::vec2 pMin, pMax;
};

class Bounds3 {
    public:
        Bounds3();
        Bounds3(const glm::vec3& p);
        Bounds3(const glm::vec3 &p1, const glm::vec3 &p2);

        const glm::vec3 &operator[](int i) const;
        glm::vec3 &operator[](int i);

        glm::vec3 corner(int corner) const;
        glm::vec3 diagonal() const;
        glm::vec3 lerp(glm::vec3& t) const;
        glm::vec3 offset(const glm::vec3& point) const;

        void boundingSphere(glm::vec3& center, float& radius) const;

        float surfaceArea() const;
        float volume() const;
        int maxExtent() const;

        glm::vec3 pMin, pMax;
};

Bounds3 Union(const Bounds3 &bounds, const glm::vec3& point);
Bounds3 Union(const Bounds3 &bounds, const Bounds3& bounds2);

Bounds3 Intersect(const Bounds3 &bounds, const Bounds3& bounds2);
bool Overlaps(const Bounds3 &bounds, const Bounds3& bounds2);

bool Inside(const glm::vec3 &point, const Bounds3& bounds2);
bool InsideExclusive(const glm::vec3 &point, const Bounds3& bounds2);

Bounds3 Expand(const Bounds3 &bounds, float delta);
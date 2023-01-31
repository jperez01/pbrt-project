#pragma once
#include <glm/glm.hpp>
#include <iostream>

#include "pbrt.h"

class Ray {
    public:
        Ray();
        Ray(const glm::vec3& org, const glm::vec3& dir, 
            float tMax = Infinity, float time = 0.f);
        
        bool hasNans();
        glm::vec3 operator()(float t);
        friend std::ostream& operator<<(std::ostream& os, const Ray r);

        glm::vec3 o;
        glm::vec3 dir;
        mutable float tMax;
        float time;
};

class RayDifferential : public Ray {
    public:
        RayDifferential();
        RayDifferential(const glm::vec3& o, const glm::vec3& d,
            float tMax = Infinity, float time = 0.f);
        RayDifferential(const Ray& ray);

        bool hasNans();
        void scaleDifferentials(float s);

        bool hasDifferentials;
        glm::vec3 rxOrigin, ryOrigin;
        glm::vec3 rxDirection, ryDirection;
};
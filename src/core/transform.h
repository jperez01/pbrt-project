#pragma once
#include "geometry.h"
#include "ray.h"
#include <glm/gtc/quaternion.hpp>

struct DerivativeTerm {
    DerivativeTerm() {
        DerivativeTerm(0, 0, 0, 0);
    }
    DerivativeTerm(float c, float x, float y, float z);
    float kc, kx, ky, kz;
    float eval(const glm::vec3& p) const {
        return kc + kx * p.x + ky * p.y + kz * p.z;
    }
};

class AnimatedTransform {
    public:
        AnimatedTransform(const glm::mat4& startTrans, const glm::mat4& endTrans,
            float startT, float endT);

        void interpolate(float time, glm::mat4& trans) const;
        Ray operator()(const Ray& r) const;
        RayDifferential operator()(const RayDifferential& r) const;
        glm::vec3 operator()(float time, const glm::vec3& vec) const;

    private:
        glm::mat4 startTransform;
        glm::mat4 endTransform;
        bool isAnimated;
        float startTime, endTime;

        glm::vec3 translations[2];
        glm::quat rotations[2];
        glm::vec3 scales[2];

        DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
};

void decomposeMatrix(const glm::mat4& matrix, glm::vec3& trans, glm::quat& rotation, glm::vec3& scale);
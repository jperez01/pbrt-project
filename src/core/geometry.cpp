#include "geometry.h"
#include <cmath>
#include <algorithm>

std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const glm::vec2& v) {
    os << "[ " << v.x << ", " << v.y << " ]";
    return os;
}

namespace pbrt {
    bool hasNan(const glm::vec3& vector) {
        return glm::any(glm::isnan(vector));
    }

    glm::vec3 faceForward(glm::vec3 normal, glm::vec3 vector) {
        return (glm::dot(normal, vector) < 0.0f) ? -normal : normal;
    }

    glm::vec3 lerp(float t, const glm::vec3& point0, const glm::vec3& point1) {
        return (1 - t) * point0 + t * point1;
    }

    float lerp(float t, float a, float b) {
        return (1 - t) * a + t * b;
    }
    
    glm::vec3 min(const glm::vec3& point1, const glm::vec3& point2) {
        return glm::vec3(std::min(point1.x, point2.x), 
            std::min(point1.y, point2.y), std::min(point1.z, point2.z));
    }
    glm::vec3 max(const glm::vec3& point1, const glm::vec3& point2) {
        return glm::vec3(std::max(point1.x, point2.x), 
            std::max(point1.y, point2.y), std::max(point1.z, point2.z));
    }

    void makeCoordSystem(glm::vec3& v1, glm::vec3& v2, glm::vec3& v3) {
        if (std::abs(v1.x) > std::abs(v1.y)) {
            v2 = glm::vec3(-v1.z, 0, v1.x) / std::sqrt(v1.x * v1.x + v1.z * v1.z);
        } else {
            v2 = glm::vec3(0, v1.z, -v1.y) / std::sqrt(v1.y * v1.y + v1.z * v1.z);
        }
        v3 = glm::cross(v1, v2);
    }
};
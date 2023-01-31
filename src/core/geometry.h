#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace pbrt {
    glm::vec3 faceForward(glm::vec3 normal, glm::vec3 vector);

    bool hasNan(const glm::vec3& vector);

    glm::vec3 lerp(float t, const glm::vec3& point0, const glm::vec3& point1);\
    float lerp(float t, float a, float b);
    glm::vec3 min(const glm::vec3& point1, const glm::vec3& point2);
    glm::vec3 max(const glm::vec3& point1, const glm::vec3& point2);
    
    void makeCoordSystem(glm::vec3& v1, glm::vec3& v2, glm::vec3& v3);
};

std::ostream& operator<<(std::ostream& os, const glm::vec3& v);
std::ostream& operator<<(std::ostream& os, const glm::vec2& v);
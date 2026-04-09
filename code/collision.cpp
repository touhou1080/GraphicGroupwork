#include "collision.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kEpsilon = 1e-6f;

float dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }

float lengthSquared(const Vec2& v) { return dot(v, v); }

float length(const Vec2& v) { return std::sqrt(lengthSquared(v)); }

Vec2 normalized(const Vec2& v) {
    const float len = length(v);
    if (len < kEpsilon) {
        return Vec2{0.0f, 0.0f};
    }
    return v / len;
}

float cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }

Vec2 cross(float s, const Vec2& v) { return Vec2{-s * v.y, s * v.x}; }

Vec2 rotate(const Vec2& v, float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return Vec2{c * v.x - s * v.y, s * v.x + c * v.y};
}

Vec2 inverseRotate(const Vec2& v, float angle) { return rotate(v, -angle); }

float clampf(float value, float minValue, float maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

Vec2 boxAxisX(const RigidBody2D& b) { return rotate(Vec2{1.0f, 0.0f}, b.angle); }
Vec2 boxAxisY(const RigidBody2D& b) { return rotate(Vec2{0.0f, 1.0f}, b.angle); }

Vec2 supportPointBox(const RigidBody2D& b, const Vec2& direction) {
    const Vec2 ax = boxAxisX(b);
    const Vec2 ay = boxAxisY(b);

    const float sx = (dot(direction, ax) >= 0.0f) ? b.shape.halfExtents.x : -b.shape.halfExtents.x;
    const float sy = (dot(direction, ay) >= 0.0f) ? b.shape.halfExtents.y : -b.shape.halfExtents.y;
    return b.position + ax * sx + ay * sy;
}

bool detectCircleCircle(const RigidBody2D& a, const RigidBody2D& b, Contact& out) {
    const Vec2 ab = b.position - a.position;
    const float distSq = lengthSquared(ab);
    const float radiusSum = a.shape.radius + b.shape.radius;
    if (distSq > radiusSum * radiusSum) {
        return false;
    }

    float dist = std::sqrt(std::max(distSq, 0.0f));
    Vec2 normal = Vec2{1.0f, 0.0f};
    if (dist > kEpsilon) {
        normal = ab / dist;
    }

    out.normal = normal;
    out.penetration = radiusSum - dist;
    out.point = a.position + normal * a.shape.radius;
    return true;
}

bool detectCircleBox(const RigidBody2D& circle, const RigidBody2D& box, Contact& out) {
    const Vec2 centerToCircle = circle.position - box.position;
    const Vec2 circleLocal = inverseRotate(centerToCircle, box.angle);

    Vec2 closestLocal{
        clampf(circleLocal.x, -box.shape.halfExtents.x, box.shape.halfExtents.x),
        clampf(circleLocal.y, -box.shape.halfExtents.y, box.shape.halfExtents.y)};

    const Vec2 delta = circleLocal - closestLocal;
    const float distSq = lengthSquared(delta);
    const float radius = circle.shape.radius;

    if (distSq > radius * radius) {
        return false;
    }

    Vec2 normalLocal;
    float penetration = 0.0f;

    if (distSq > kEpsilon) {
        const float dist = std::sqrt(distSq);
        // A->B normal (circle->box) is opposite to box->circle direction.
        normalLocal = delta * (-1.0f / dist);
        penetration = radius - dist;
    } else {
        // Circle center is inside box local bounds.
        const float dx = box.shape.halfExtents.x - std::abs(circleLocal.x);
        const float dy = box.shape.halfExtents.y - std::abs(circleLocal.y);

        if (dx < dy) {
            const float sign = (circleLocal.x >= 0.0f) ? 1.0f : -1.0f;
            normalLocal = Vec2{-sign, 0.0f};
            closestLocal = Vec2{sign * box.shape.halfExtents.x, circleLocal.y};
            penetration = radius + dx;
        } else {
            const float sign = (circleLocal.y >= 0.0f) ? 1.0f : -1.0f;
            normalLocal = Vec2{0.0f, -sign};
            closestLocal = Vec2{circleLocal.x, sign * box.shape.halfExtents.y};
            penetration = radius + dy;
        }
    }

    out.normal = rotate(normalLocal, box.angle);
    out.penetration = penetration;
    out.point = box.position + rotate(closestLocal, box.angle);
    return true;
}

bool detectBoxBoxSAT(const RigidBody2D& a, const RigidBody2D& b, Contact& out) {
    const Vec2 axes[4] = {boxAxisX(a), boxAxisY(a), boxAxisX(b), boxAxisY(b)};
    const Vec2 aAxes[2] = {boxAxisX(a), boxAxisY(a)};
    const Vec2 bAxes[2] = {boxAxisX(b), boxAxisY(b)};
    const Vec2 t = b.position - a.position;

    float minOverlap = 1e30f;
    Vec2 bestAxis = Vec2{1.0f, 0.0f};

    for (const Vec2& axis : axes) {
        const float dist = std::abs(dot(t, axis));
        const float projA =
            a.shape.halfExtents.x * std::abs(dot(aAxes[0], axis)) +
            a.shape.halfExtents.y * std::abs(dot(aAxes[1], axis));
        const float projB =
            b.shape.halfExtents.x * std::abs(dot(bAxes[0], axis)) +
            b.shape.halfExtents.y * std::abs(dot(bAxes[1], axis));

        const float overlap = projA + projB - dist;
        if (overlap <= 0.0f) {
            return false;
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            bestAxis = (dot(t, axis) >= 0.0f) ? axis : axis * -1.0f;
        }
    }

    out.normal = normalized(bestAxis);
    out.penetration = minOverlap;

    // Approximate manifold point by averaging support points from both bodies.
    const Vec2 supportA = supportPointBox(a, out.normal);
    const Vec2 supportB = supportPointBox(b, out.normal * -1.0f);
    out.point = (supportA + supportB) * 0.5f;
    return true;
}

bool detectPair(const RigidBody2D& a, const RigidBody2D& b, Contact& out) {
    if (a.shape.type == ShapeType::Circle && b.shape.type == ShapeType::Circle) {
        return detectCircleCircle(a, b, out);
    }

    if (a.shape.type == ShapeType::Circle && b.shape.type == ShapeType::Box) {
        return detectCircleBox(a, b, out);
    }

    if (a.shape.type == ShapeType::Box && b.shape.type == ShapeType::Circle) {
        if (!detectCircleBox(b, a, out)) {
            return false;
        }
        out.normal = out.normal * -1.0f;
        return true;
    }

    return detectBoxBoxSAT(a, b, out);
}
}  // namespace

void detectCollisions(const std::vector<RigidBody2D>& bodies, std::vector<Contact>& outContacts) {
    outContacts.clear();

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            if (bodies[i].isStatic && bodies[j].isStatic) {
                continue;
            }

            Contact contact;
            if (detectPair(bodies[i], bodies[j], contact)) {
                contact.bodyA = i;
                contact.bodyB = j;
                outContacts.push_back(contact);
            }
        }
    }
}

void resolveCollisions(std::vector<RigidBody2D>& bodies, const std::vector<Contact>& contacts) {
    constexpr float kPositionCorrectionPercent = 0.8f;
    constexpr float kPositionSlop = 0.01f;

    for (const Contact& contact : contacts) {
        RigidBody2D& a = bodies[contact.bodyA];
        RigidBody2D& b = bodies[contact.bodyB];

        if (a.isStatic && b.isStatic) {
            continue;
        }

        const Vec2 ra = contact.point - a.position;
        const Vec2 rb = contact.point - b.position;

        const Vec2 va = a.velocity + cross(a.angularVelocity, ra);
        const Vec2 vb = b.velocity + cross(b.angularVelocity, rb);
        const Vec2 relativeVelocity = vb - va;
        const float velAlongNormal = dot(relativeVelocity, contact.normal);

        if (velAlongNormal < 0.0f) {
            const float restitution = std::min(a.restitution, b.restitution);
            const float raCrossN = cross(ra, contact.normal);
            const float rbCrossN = cross(rb, contact.normal);
            const float invMassSum =
                a.invMass + b.invMass + raCrossN * raCrossN * a.invInertia +
                rbCrossN * rbCrossN * b.invInertia;

            if (invMassSum > kEpsilon) {
                const float j = -(1.0f + restitution) * velAlongNormal / invMassSum;
                const Vec2 impulse = contact.normal * j;

                a.velocity -= impulse * a.invMass;
                b.velocity += impulse * b.invMass;
                a.angularVelocity -= raCrossN * j * a.invInertia;
                b.angularVelocity += rbCrossN * j * b.invInertia;
            }
        }

        const float invMassForCorrection = a.invMass + b.invMass;
        if (invMassForCorrection > kEpsilon) {
            const float correctionMagnitude =
                std::max(contact.penetration - kPositionSlop, 0.0f) *
                (kPositionCorrectionPercent / invMassForCorrection);
            const Vec2 correction = contact.normal * correctionMagnitude;

            a.position -= correction * a.invMass;
            b.position += correction * b.invMass;
        }
    }
}

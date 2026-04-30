#include "collision.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

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

std::array<Vec2, 4> getBoxVertices(const RigidBody2D& b) {
    const Vec2 ax = boxAxisX(b);
    const Vec2 ay = boxAxisY(b);
    const float hx = b.shape.halfExtents.x;
    const float hy = b.shape.halfExtents.y;
    return {b.position + ax * hx + ay * hy,
            b.position - ax * hx + ay * hy,
            b.position - ax * hx - ay * hy,
            b.position + ax * hx - ay * hy};
}

struct Face {
    Vec2 v1{0.0f, 0.0f};
    Vec2 v2{0.0f, 0.0f};
    Vec2 normal{0.0f, 1.0f};
};

Face getFaceTowardNormal(const RigidBody2D& box, const Vec2& normal) {
    const Vec2 ax = boxAxisX(box);
    const Vec2 ay = boxAxisY(box);
    const float dx = dot(normal, ax);
    const float dy = dot(normal, ay);

    Face face;
    if (std::abs(dx) > std::abs(dy)) {
        const float sign = (dx >= 0.0f) ? 1.0f : -1.0f;
        const Vec2 faceCenter = box.position + ax * (sign * box.shape.halfExtents.x);
        face.normal = ax * sign;
        face.v1 = faceCenter + ay * box.shape.halfExtents.y;
        face.v2 = faceCenter - ay * box.shape.halfExtents.y;
    } else {
        const float sign = (dy >= 0.0f) ? 1.0f : -1.0f;
        const Vec2 faceCenter = box.position + ay * (sign * box.shape.halfExtents.y);
        face.normal = ay * sign;
        face.v1 = faceCenter - ax * box.shape.halfExtents.x;
        face.v2 = faceCenter + ax * box.shape.halfExtents.x;
    }
    return face;
}

Face getIncidentFace(const RigidBody2D& box, const Vec2& referenceNormal) {
    return getFaceTowardNormal(box, referenceNormal * -1.0f);
}

int clipSegmentToLine(Vec2 outPoints[2], const Vec2 inPoints[2], const Vec2& normal, float offset) {
    int outCount = 0;

    const float d0 = dot(normal, inPoints[0]) - offset;
    const float d1 = dot(normal, inPoints[1]) - offset;

    if (d0 <= 0.0f) {
        outPoints[outCount++] = inPoints[0];
    }
    if (d1 <= 0.0f) {
        outPoints[outCount++] = inPoints[1];
    }

    if (d0 * d1 < 0.0f && outCount < 2) {
        const float t = d0 / (d0 - d1);
        outPoints[outCount++] = inPoints[0] + (inPoints[1] - inPoints[0]) * t;
    }

    return outCount;
}

void fallbackBoxContact(const RigidBody2D& a, const RigidBody2D& b, Contact& out) {
    const Vec2 supportA = supportPointBox(a, out.normal);
    const Vec2 supportB = supportPointBox(b, out.normal * -1.0f);
    out.points[0] = (supportA + supportB) * 0.5f;
    out.pointCount = 1;
}

bool pointInsideBox(const Vec2& p, const RigidBody2D& b) {
    const Vec2 local = inverseRotate(p - b.position, b.angle);
    constexpr float kTol = 0.01f;
    return std::abs(local.x) <= b.shape.halfExtents.x + kTol &&
           std::abs(local.y) <= b.shape.halfExtents.y + kTol;
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
    out.points[0] = a.position + normal * a.shape.radius;
    out.pointCount = 1;
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
        normalLocal = delta * (-1.0f / dist);
        penetration = radius - dist;
    } else {
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
    out.points[0] = box.position + rotate(closestLocal, box.angle);
    out.pointCount = 1;
    return true;
}
/*
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

    std::vector<Vec2> candidates;
    candidates.reserve(8);

    const auto vertsA = getBoxVertices(a);
    const auto vertsB = getBoxVertices(b);

    for (const Vec2& v : vertsA) {
        if (pointInsideBox(v, b)) {
            candidates.push_back(v);
        }
    }
    for (const Vec2& v : vertsB) {
        if (pointInsideBox(v, a)) {
            candidates.push_back(v);
        }
    }

    if (candidates.empty()) {
        const Vec2 supportA = supportPointBox(a, out.normal);
        const Vec2 supportB = supportPointBox(b, out.normal * -1.0f);
        out.points[0] = (supportA + supportB) * 0.5f;
        out.pointCount = 1;
        return true;
    }

    const Vec2 tangent{-out.normal.y, out.normal.x};
    float minProj = 1e30f;
    float maxProj = -1e30f;
    Vec2 minPoint = candidates[0];
    Vec2 maxPoint = candidates[0];

    for (const Vec2& p : candidates) {
        const float proj = dot(p, tangent);
        if (proj < minProj) {
            minProj = proj;
            minPoint = p;
        }
        if (proj > maxProj) {
            maxProj = proj;
            maxPoint = p;
        }
    }

    out.points[0] = minPoint;
    out.pointCount = 1;

    if (lengthSquared(maxPoint - minPoint) > 1e-4f) {
        out.points[1] = maxPoint;
        out.pointCount = 2;
    }

    return true;
}
*/

bool detectBoxBoxSAT(const RigidBody2D& a, const RigidBody2D& b, Contact& out) {
    const Vec2 axes[4] = {boxAxisX(a), boxAxisY(a), boxAxisX(b), boxAxisY(b)};
    const Vec2 aAxes[2] = {boxAxisX(a), boxAxisY(a)};
    const Vec2 bAxes[2] = {boxAxisX(b), boxAxisY(b)};
    const Vec2 t = b.position - a.position;

    float minOverlap = 1e30f;
    Vec2 bestAxis = Vec2{1.0f, 0.0f};
    int bestAxisIndex = 0;  // 0:A.x, 1:A.y, 2:B.x, 3:B.y

    for (int i = 0; i < 4; ++i) {
        const Vec2& axis = axes[i];
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
            bestAxisIndex = i;
            bestAxis = (dot(t, axis) >= 0.0f) ? axis : axis * -1.0f;
        }
    }

    out.normal = normalized(bestAxis);  // Always points from A to B.
    out.penetration = minOverlap;
    out.pointCount = 0;

    const bool referenceIsA = (bestAxisIndex < 2);
    const RigidBody2D& refBox = referenceIsA ? a : b;
    const RigidBody2D& incBox = referenceIsA ? b : a;

    // If B supplies the reference face, use the opposite normal for B's local
    // reference face. The public contact normal still remains A -> B.
    const Vec2 referenceNormal = referenceIsA ? out.normal : out.normal * -1.0f;
    const Face refFace = getFaceTowardNormal(refBox, referenceNormal);
    const Face incFace = getIncidentFace(incBox, refFace.normal);

    const Vec2 refEdge = normalized(refFace.v2 - refFace.v1);
    if (lengthSquared(refEdge) < kEpsilon) {
        fallbackBoxContact(a, b, out);
        return true;
    }

    Vec2 incident[2] = {incFace.v1, incFace.v2};
    Vec2 clip1[2];
    Vec2 clip2[2];

    const Vec2 sideNormal1 = refEdge * -1.0f;
    const float sideOffset1 = dot(sideNormal1, refFace.v1);
    const Vec2 sideNormal2 = refEdge;
    const float sideOffset2 = dot(sideNormal2, refFace.v2);

    int count = clipSegmentToLine(clip1, incident, sideNormal1, sideOffset1);
    if (count < 2) {
        fallbackBoxContact(a, b, out);
        return true;
    }

    count = clipSegmentToLine(clip2, clip1, sideNormal2, sideOffset2);
    if (count < 2) {
        fallbackBoxContact(a, b, out);
        return true;
    }

    const float referenceOffset = dot(refFace.normal, refFace.v1);
    constexpr float kContactTolerance = 0.01f;

    for (int i = 0; i < count && out.pointCount < 2; ++i) {
        const float separation = dot(refFace.normal, clip2[i]) - referenceOffset;
        if (separation <= kContactTolerance) {
            // Project the point back onto the reference face so the solver uses
            // a boundary point rather than a deeply buried point as the force arm.
            out.points[out.pointCount++] = clip2[i] - refFace.normal * separation;
        }
    }

    if (out.pointCount == 0) {
        fallbackBoxContact(a, b, out);
    }

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

struct SolverContactPoint {
    Vec2 point{0.0f, 0.0f};
    Vec2 localPointA{0.0f, 0.0f};
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
    // Target separating speed (= -restitution * initial velAlongNormal, clamped to >= 0).
    // Computed ONCE before the iteration loop using initial velocities so
    // restitution does not get cancelled out in later iterations.
    float velocityBias = 0.0f;
};

struct SolverContact {
    std::size_t bodyA = 0;
    std::size_t bodyB = 0;
    Vec2 normal{0.0f, 1.0f};
    float penetration = 0.0f;
    int pointCount = 0;
    SolverContactPoint points[2];
};

void applyImpulseAtPoint(RigidBody2D& body, const Vec2& impulse, const Vec2& r) {
    if (body.isStatic) {
        return;
    }
    body.velocity += impulse * body.invMass;
    body.angularVelocity += cross(r, impulse) * body.invInertia;
}

const CachedContactImpulse* findCachedImpulse(const std::vector<CachedContactImpulse>& cache,
                                              std::size_t bodyA,
                                              std::size_t bodyB,
                                              const Vec2& localPointA) {
    constexpr float kMatchDistanceSq = 0.04f * 0.04f;

    const CachedContactImpulse* best = nullptr;
    float bestDistanceSq = kMatchDistanceSq;
    for (const CachedContactImpulse& cached : cache) {
        if (cached.bodyA != bodyA || cached.bodyB != bodyB) {
            continue;
        }

        const float distanceSq = lengthSquared(cached.localPointA - localPointA);
        if (distanceSq <= bestDistanceSq) {
            best = &cached;
            bestDistanceSq = distanceSq;
        }
    }
    return best;
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

void resolveCollisions(std::vector<RigidBody2D>& bodies,
                       const std::vector<Contact>& contacts,
                       float dt,
                       std::vector<CachedContactImpulse>& cache) {
    constexpr int kVelocityIterations = 8;
    constexpr float kPositionCorrectionPercent = 0.2f;
    constexpr float kPositionSlop = 0.01f;
    constexpr float kWakeSpeedSq = 0.02f * 0.02f;
    constexpr float kBaumgarte = 0.08f;
    constexpr float kMaxBias = 2.0f;

    std::vector<SolverContact> solverContacts;
    solverContacts.reserve(contacts.size());

    for (const Contact& contact : contacts) {
        if (contact.pointCount <= 0) {
            continue;
        }

        SolverContact sc;
        sc.bodyA = contact.bodyA;
        sc.bodyB = contact.bodyB;
        sc.normal = contact.normal;
        sc.penetration = contact.penetration;
        sc.pointCount = contact.pointCount;
        for (int i = 0; i < contact.pointCount; ++i) {
            SolverContactPoint& cp = sc.points[i];
            cp.point = contact.points[i];
            cp.localPointA = inverseRotate(cp.point - bodies[contact.bodyA].position,
                                           bodies[contact.bodyA].angle);

            if (const CachedContactImpulse* cached =
                    findCachedImpulse(cache, contact.bodyA, contact.bodyB, cp.localPointA)) {
                cp.normalImpulse = cached->normalImpulse;
                cp.tangentImpulse = cached->tangentImpulse;
            }
        }
        solverContacts.push_back(sc);
    }

    for (const SolverContact& contact : solverContacts) {
        RigidBody2D& a = bodies[contact.bodyA];
        RigidBody2D& b = bodies[contact.bodyB];

        if (a.isStatic && b.isStatic) {
            continue;
        }

        const float speedSqA = lengthSquared(a.velocity) + a.angularVelocity * a.angularVelocity;
        const float speedSqB = lengthSquared(b.velocity) + b.angularVelocity * b.angularVelocity;
        if (!a.isStatic && (contact.penetration > kPositionSlop || speedSqB > kWakeSpeedSq)) {
            a.isSleeping = false;
            a.sleepTimer = 0.0f;
        }
        if (!b.isStatic && (contact.penetration > kPositionSlop || speedSqA > kWakeSpeedSq)) {
            b.isSleeping = false;
            b.sleepTimer = 0.0f;
        }
    }

    // Pre-pass: compute the restitution velocity bias from the initial relative
    // velocity at each contact point. Doing this once (not per iteration) is what
    // makes "bouncy" bodies actually bounce — otherwise iteration 2 sees the
    // post-impact separating velocity and reels the impulse back, killing the bounce.
    constexpr float kRestitutionThreshold = 0.5f;
    for (SolverContact& contact : solverContacts) {
        const RigidBody2D& a = bodies[contact.bodyA];
        const RigidBody2D& b = bodies[contact.bodyB];
        const float restitution = std::max(a.restitution, b.restitution);
        for (int p = 0; p < contact.pointCount; ++p) {
            SolverContactPoint& cp = contact.points[p];
            const Vec2 ra = cp.point - a.position;
            const Vec2 rb = cp.point - b.position;
            const Vec2 va = a.velocity + cross(a.angularVelocity, ra);
            const Vec2 vb = b.velocity + cross(b.angularVelocity, rb);
            const float vn = dot(vb - va, contact.normal);
            cp.velocityBias = (vn < -kRestitutionThreshold) ? -restitution * vn : 0.0f;
        }
    }

    for (SolverContact& contact : solverContacts) {
        RigidBody2D& a = bodies[contact.bodyA];
        RigidBody2D& b = bodies[contact.bodyB];

        if (a.isStatic && b.isStatic) {
            continue;
        }

        const Vec2 tangent{-contact.normal.y, contact.normal.x};
        for (int p = 0; p < contact.pointCount; ++p) {
            SolverContactPoint& cp = contact.points[p];
            const Vec2 ra = cp.point - a.position;
            const Vec2 rb = cp.point - b.position;
            const Vec2 impulse = contact.normal * cp.normalImpulse + tangent * cp.tangentImpulse;
            applyImpulseAtPoint(a, impulse * -1.0f, ra);
            applyImpulseAtPoint(b, impulse, rb);
        }
    }

    for (int iter = 0; iter < kVelocityIterations; ++iter) {
        for (SolverContact& contact : solverContacts) {
            RigidBody2D& a = bodies[contact.bodyA];
            RigidBody2D& b = bodies[contact.bodyB];

            if (a.isStatic && b.isStatic) {
                continue;
            }

            const float muS = std::sqrt(a.staticFriction * b.staticFriction);
            const float muD = std::sqrt(a.dynamicFriction * b.dynamicFriction);
            const Vec2 tangentBase{-contact.normal.y, contact.normal.x};

            for (int p = 0; p < contact.pointCount; ++p) {
                SolverContactPoint& cp = contact.points[p];
                const Vec2 ra = cp.point - a.position;
                const Vec2 rb = cp.point - b.position;

                const Vec2 va = a.velocity + cross(a.angularVelocity, ra);
                const Vec2 vb = b.velocity + cross(b.angularVelocity, rb);
                const Vec2 rv = vb - va;

                const float velAlongNormal = dot(rv, contact.normal);
                const float raCrossN = cross(ra, contact.normal);
                const float rbCrossN = cross(rb, contact.normal);
                const float normalMass =
                    a.invMass + b.invMass + raCrossN * raCrossN * a.invInertia +
                    rbCrossN * rbCrossN * b.invInertia;

                if (normalMass > kEpsilon) {
                    float bias = 0.0f;
                    if (contact.penetration > kPositionSlop) {
                        const float pointShare = 1.0f / static_cast<float>(std::max(contact.pointCount, 1));
                        const float safeDt = std::max(dt, kEpsilon);
                        bias = (kBaumgarte / safeDt) * (contact.penetration - kPositionSlop) * pointShare;
                        bias = std::min(bias, kMaxBias);
                        //bias = 0.15f * (contact.penetration - kPositionSlop) * pointShare;
                        //bias = 0.15f * (contact.penetration - kPositionSlop);
                    }

                    // Drive the contact's normal velocity toward the precomputed
                    // separation target (cp.velocityBias >= 0). For a fresh impact
                    // the target is +restitution * incomingSpeed; otherwise it's 0.
                    float lambda = -(velAlongNormal - bias - cp.velocityBias) / normalMass;
                    const float oldImpulse = cp.normalImpulse;
                    cp.normalImpulse = std::max(oldImpulse + lambda, 0.0f);
                    lambda = cp.normalImpulse - oldImpulse;

                    const Vec2 impulse = contact.normal * lambda;
                    applyImpulseAtPoint(a, impulse * -1.0f, ra);
                    applyImpulseAtPoint(b, impulse, rb);
                }

                const Vec2 vaAfterN = a.velocity + cross(a.angularVelocity, ra);
                const Vec2 vbAfterN = b.velocity + cross(b.angularVelocity, rb);
                const Vec2 rvAfterN = vbAfterN - vaAfterN;

                Vec2 tangent = tangentBase;
                const float tangentSpeed = dot(rvAfterN, tangent);
                const float raCrossT = cross(ra, tangent);
                const float rbCrossT = cross(rb, tangent);
                const float tangentMass =
                    a.invMass + b.invMass + raCrossT * raCrossT * a.invInertia +
                    rbCrossT * rbCrossT * b.invInertia;

                if (tangentMass > kEpsilon) {
                    float lambdaT = -tangentSpeed / tangentMass;
                    const float maxFriction = muD * cp.normalImpulse;
                    const float oldTangentImpulse = cp.tangentImpulse;
                    const float candidate = oldTangentImpulse + lambdaT;
                    cp.tangentImpulse = clampf(candidate, -maxFriction, maxFriction);
                    lambdaT = cp.tangentImpulse - oldTangentImpulse;

                    if (std::abs(candidate) <= muS * cp.normalImpulse) {
                        cp.tangentImpulse = candidate;
                        lambdaT = cp.tangentImpulse - oldTangentImpulse;
                    }

                    const Vec2 frictionImpulse = tangent * lambdaT;
                    applyImpulseAtPoint(a, frictionImpulse * -1.0f, ra);
                    applyImpulseAtPoint(b, frictionImpulse, rb);
                }
            }
        }
    }

    for (const SolverContact& contact : solverContacts) {
        RigidBody2D& a = bodies[contact.bodyA];
        RigidBody2D& b = bodies[contact.bodyB];

        const float invMassForCorrection = a.invMass + b.invMass;
        if (invMassForCorrection <= kEpsilon) {
            continue;
        }

        const float correctionMagnitude =
            std::max(contact.penetration - kPositionSlop, 0.0f) *
            (kPositionCorrectionPercent / invMassForCorrection);
        const Vec2 correction = contact.normal * correctionMagnitude;

        if (!a.isStatic) {
            a.position -= correction * a.invMass;
        }
        if (!b.isStatic) {
            b.position += correction * b.invMass;
        }
    }

    cache.clear();
    for (const SolverContact& contact : solverContacts) {
        for (int p = 0; p < contact.pointCount; ++p) {
            const SolverContactPoint& cp = contact.points[p];
            CachedContactImpulse cached;
            cached.bodyA = contact.bodyA;
            cached.bodyB = contact.bodyB;
            cached.localPointA = cp.localPointA;
            cached.normalImpulse = cp.normalImpulse;
            cached.tangentImpulse = cp.tangentImpulse;
            cache.push_back(cached);
        }
    }
}

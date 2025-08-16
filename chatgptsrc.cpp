#include <SFML/Graphics.hpp>
#include <optional>
#include <vector>
#include <cmath>
#include <algorithm>

// ---------- Math helpers (2D) ----------
static inline float dot(const sf::Vector2f& a, const sf::Vector2f& b) { return a.x*b.x + a.y*b.y; }
static inline float length(const sf::Vector2f& v) { return std::sqrt(dot(v,v)); }
static inline sf::Vector2f normalize(const sf::Vector2f& v) {
    float l = length(v); return (l > 0.f) ? (v / l) : sf::Vector2f(0.f,0.f);
}
// 2D "cross products":
// scalar = cross(a, b) -> z-component of 3D cross (a.x*b.y - a.y*b.x)
static inline float cross(const sf::Vector2f& a, const sf::Vector2f& b) { return a.x*b.y - a.y*b.x; }
// vector = cross(a, s) => rotate a by 90° CCW then scale by s
static inline sf::Vector2f cross(const sf::Vector2f& a, float s) { return sf::Vector2f( s * a.y, -s * a.x ); }
// vector = cross(s, a) => rotate a by 90° CW then scale by s
static inline sf::Vector2f cross(float s, const sf::Vector2f& a) { return sf::Vector2f( -s * a.y, s * a.x ); }

// ---------- Rigid body (rectangle/OBB) ----------
struct RigidBody {
    // State
    sf::Vector2f pos{0,0};
    float angle{0.f};                 // radians
    sf::Vector2f vel{0,0};
    float angVel{0.f};                // rad/s

    // Physical props
    float mass{1.f}, invMass{1.f};
    float inertia{1.f}, invInertia{1.f}; // about center
    float restitution{0.2f};          // bounciness
    float friction{0.6f};             // Coulomb friction coefficient

    // Box shape
    sf::Vector2f halfExtents{100.f,20.f}; // 200x40
    sf::RectangleShape drawable;

    bool isStatic{false};

    void makeBox(const sf::Vector2f& center, const sf::Vector2f& size, float density, bool makeStatic=false, sf::Color color=sf::Color::White) {
        pos = center;
        angle = 0.f;
        halfExtents = size * 0.5f;
        isStatic = makeStatic;

        float w = size.x, h = size.y;
        if (makeStatic) {
            mass = INFINITY; invMass = 0.f;
            inertia = INFINITY; invInertia = 0.f;
        } else {
            // mass = density * area (simple 2D)
            mass = density * w * h;
            invMass = 1.f / mass;
            // rectangle inertia about center: (1/12) m (w^2 + h^2)
            inertia = (mass * (w*w + h*h)) / 12.f;
            invInertia = 1.f / inertia;
        }

        drawable.setSize(size);
        drawable.setOrigin(sf::Vector2f(size.x*0.5f, size.y*0.5f));
        drawable.setFillColor(color);
        drawable.setPosition(pos);
        drawable.setRotation(sf::radians(angle));
    }

    // World-space corners (CCW)
    void getWorldVerts(sf::Vector2f out[4]) const {
        const sf::Vector2f hx = { halfExtents.x, 0.f };
        const sf::Vector2f hy = { 0.f, halfExtents.y };
        // local corners
        sf::Vector2f local[4] = {
            {-halfExtents.x, -halfExtents.y},
            { halfExtents.x, -halfExtents.y},
            { halfExtents.x,  halfExtents.y},
            {-halfExtents.x,  halfExtents.y},
        };
        float c = std::cos(angle), s = std::sin(angle);
        for (int i=0;i<4;++i){
            sf::Vector2f r = { c*local[i].x - s*local[i].y, s*local[i].x + c*local[i].y };
            out[i] = pos + r;
        }
    }

    // Apply a force at a world-space point (accumulates instantaneous impulse-style for this demo)
    void applyImpulseAtPoint(const sf::Vector2f& impulse, const sf::Vector2f& worldPoint) {
        if (invMass == 0.f) return; // static
        vel += impulse * invMass;
        sf::Vector2f r = worldPoint - pos;
        angVel += invInertia * cross(r, impulse);
    }

    // Simple integration
    void integrate(const sf::Vector2f& gravity, float linearDampingPerSec, float angularDampingPerSec, float dt) {
        if (invMass == 0.f) { // static
            drawable.setPosition(pos);
            drawable.setRotation(sf::degrees((angle * 180.f / 3.1415926535f)));
            return;
        }
        // Gravity
        vel += gravity * dt;

        // Damping (friction-like air drag)
        float linKeep = std::pow(linearDampingPerSec, dt);   // e.g., 0.6^1s -> 40% loss/sec
        float angKeep = std::pow(angularDampingPerSec, dt);
        vel *= linKeep;
        angVel *= angKeep;

        // Integrate
        pos += vel * dt;
        angle += angVel * dt;

        drawable.setPosition(pos);
        drawable.setRotation(sf::degrees(angle * 180.f / 3.1415926535f));
    }
};

// ---------- SAT / collision ----------
struct Manifold {
    bool colliding{false};
    sf::Vector2f normal{0,0}; // from A to B
    float penetration{0.f};
    // up to 2 contacts for box-box
    sf::Vector2f contacts[2];
    int contactCount{0};
};

// Project polygon on axis
static void projectOnto(const sf::Vector2f pts[4], const sf::Vector2f& axis, float& minOut, float& maxOut) {
    minOut = maxOut = dot(pts[0], axis);
    for (int i=1;i<4;++i) {
        float d = dot(pts[i], axis);
        if (d < minOut) minOut = d;
        if (d > maxOut) maxOut = d;
    }
}

static bool overlapOnAxis(const sf::Vector2f A[4], const sf::Vector2f B[4], const sf::Vector2f& axis, float& overlap) {
    // axis should be normalized
    float minA, maxA, minB, maxB;
    projectOnto(A, axis, minA, maxA);
    projectOnto(B, axis, minB, maxB);
    if (maxA < minB || maxB < minA) return false;
    overlap = std::min(maxA, maxB) - std::max(minA, minB);
    return true;
}

static sf::Vector2f edgeNormal(const sf::Vector2f& p0, const sf::Vector2f& p1){
    sf::Vector2f e = p1 - p0;
    sf::Vector2f n = { -e.y, e.x };
    float L = length(n); return (L>0)?(n/L):sf::Vector2f(0,0);
}

// Clip a segment to half-space
static int clip(const sf::Vector2f& n, float c, sf::Vector2f* face) {
    sf::Vector2f out[2] = {face[0], face[1]};
    int sp = 0;
    float d1 = dot(n, face[0]) - c;
    float d2 = dot(n, face[1]) - c;

    if (d1 <= 0.f) out[sp++] = face[0];
    if (d2 <= 0.f) out[sp++] = face[1];

    if (d1 * d2 < 0.f) {
        // Edge intersects
        float alpha = d1 / (d1 - d2);
        out[sp++] = face[0] + alpha * (face[1] - face[0]);
    }
    face[0] = out[0];
    face[1] = out[1];
    return sp;
}

// Find reference and incident faces and build contacts (box-box, SAT)
static void findContacts(const RigidBody& A, const RigidBody& B, const sf::Vector2f& normal, Manifold& m) {
    // Build polygons
    sf::Vector2f a[4], b[4]; A.getWorldVerts(a); B.getWorldVerts(b);

    // Pick reference face on the box with face most aligned with the collision normal (smallest angle)
    auto bestFace = [](const sf::Vector2f pts[4], const sf::Vector2f& n) {
        int best = 0; float bestDot = dot(edgeNormal(pts[0], pts[1]), n);
        float d1 = std::fabs(bestDot);
        best = 0;
        float minAngle = 1e9; int idx = 0;
        for (int i=0;i<4;++i) {
            sf::Vector2f en = edgeNormal(pts[i], pts[(i+1)&3]); // outward normal
            float ang = std::acos(std::clamp(dot(en, n), -1.f, 1.f));
            if (ang < minAngle) { minAngle = ang; idx = i; }
        }
        return idx;
    };

    // Decide reference on the shape with larger projection along normal
    // Simpler rule: choose reference on A if penetration came from A's axes; we don't have that flag here.
    // We'll choose the box whose face normal is closer to 'normal' as reference.
    int refA = bestFace(a, normal);
    int refB = bestFace(b, -normal);

    // Compare alignment to choose reference shape
    float alignA = std::fabs(dot(edgeNormal(a[refA], a[(refA+1)&3]), normal));
    float alignB = std::fabs(dot(edgeNormal(b[refB], b[(refB+1)&3]), -normal));
    bool AisRef = alignA >= alignB;

    const sf::Vector2f* ref = AisRef ? a : b;
    const sf::Vector2f* inc = AisRef ? b : a;
    sf::Vector2f refN = AisRef ? normal : -normal;

    int iRef = AisRef ? refA : refB;

    // Reference face endpoints
    sf::Vector2f v1 = ref[iRef];
    sf::Vector2f v2 = ref[(iRef+1)&3];
    sf::Vector2f sideDir = normalize(v2 - v1);
    sf::Vector2f refFaceNormal = { sideDir.y, -sideDir.x }; // outward

    // Find incident face: edge on incident most anti-parallel to ref normal
    int iInc = 0;
    {
        float minDot = 1e9;
        for (int i=0;i<4;++i) {
            sf::Vector2f en = edgeNormal(inc[i], inc[(i+1)&3]);
            float d = dot(en, refFaceNormal);
            if (d < minDot) { minDot = d; iInc = i; }
        }
    }

    sf::Vector2f incFace[2] = { inc[iInc], inc[(iInc+1)&3] };

    // Clip incident against reference side planes
    float refC = dot(refFaceNormal, v1);
    // First clip against side plane 1
    if (clip( sideDir, dot(sideDir, v1), incFace ) < 2) return;
    // Then against opposite side plane
    if (clip(-sideDir, -dot(sideDir, v2), incFace) < 2) return;

    // Finally, keep points behind reference face
    float refN_c = dot(refFaceNormal, v1);
    sf::Vector2f outPts[2] = {incFace[0], incFace[1]};
    int cp = 0;
    for (int i=0;i<2;++i) {
        float sep = dot(refFaceNormal, incFace[i]) - refN_c;
        if (sep <= 0.f) {
            outPts[cp++] = incFace[i];
        }
    }
    m.contactCount = cp;
    for (int i=0;i<cp;++i) m.contacts[i] = outPts[i];
    m.normal = AisRef ? refFaceNormal : -refFaceNormal;
}

// SAT test + manifold (normal points from A to B)
static Manifold collideOBB(const RigidBody& A, const RigidBody& B) {
    Manifold m;
    sf::Vector2f a[4], b[4]; A.getWorldVerts(a); B.getWorldVerts(b);

    // Candidate axes: 2 from A, 2 from B (box)
    sf::Vector2f axes[4] = {
        edgeNormal(a[0], a[1]),
        edgeNormal(a[1], a[2]),
        edgeNormal(b[0], b[1]),
        edgeNormal(b[1], b[2])
    };

    float minOverlap = 1e9;
    sf::Vector2f bestAxis;

    for (int i=0;i<4;++i) {
        sf::Vector2f axis = axes[i];
        float ov;
        if (!overlapOnAxis(a, b, axis, ov)) {
            m.colliding = false;
            return m;
        }
        if (ov < minOverlap) {
            minOverlap = ov;
            bestAxis = axis;
        }
    }

    // Ensure axis points from A to B
    sf::Vector2f d = B.pos - A.pos;
    if (dot(d, bestAxis) < 0.f) bestAxis = -bestAxis;

    m.colliding = true;
    m.normal = bestAxis;
    m.penetration = minOverlap;

    // Build contacts by clipping incident face to reference face
    findContacts(A, B, m.normal, m);
    if (m.contactCount == 0) {
        // Fallback single contact at midpoint along normal
        m.contacts[0] = A.pos + 0.5f * m.normal * m.penetration;
        m.contactCount = 1;
    }
    return m;
}

// Resolve collision with impulses (restitution + friction)
static void resolveCollision(RigidBody& A, RigidBody& B, const Manifold& m) {
    if (!m.colliding) return;
    sf::Vector2f normal = m.normal;

    // Positional correction (prevent sinking)
    const float percent = 0.4f; // 40% of penetration
    const float slop = 0.01f;
    float corrMag = std::max(m.penetration - slop, 0.f) * percent;
    sf::Vector2f correction = (corrMag / (A.invMass + B.invMass)) * normal;
    if (A.invMass > 0) A.pos -= correction * A.invMass;
    if (B.invMass > 0) B.pos += correction * B.invMass;

    float e = std::min(A.restitution, B.restitution);
    float mu = std::sqrt(A.friction * B.friction); // combine frictions

    for (int i=0;i<m.contactCount;++i) {
        sf::Vector2f ra = m.contacts[i] - A.pos;
        sf::Vector2f rb = m.contacts[i] - B.pos;

        sf::Vector2f rv = (B.vel + cross(B.angVel, rb)) - (A.vel + cross(A.angVel, ra));
        float velAlongNormal = dot(rv, normal);
        if (velAlongNormal > 0.f) continue; // separating

        float raN = cross(ra, normal);
        float rbN = cross(rb, normal);
        float invMassSum = A.invMass + B.invMass + (raN*raN)*A.invInertia + (rbN*rbN)*B.invInertia;

        float j = -(1.f + e) * velAlongNormal;
        j /= invMassSum;
        j /= static_cast<float>(m.contactCount); // distribute across contacts

        sf::Vector2f impulse = j * normal;
        if (A.invMass > 0) {
            A.vel -= impulse * A.invMass;
            A.angVel -= A.invInertia * cross(ra, impulse);
        }
        if (B.invMass > 0) {
            B.vel += impulse * B.invMass;
            B.angVel += B.invInertia * cross(rb, impulse);
        }

        // Friction
        sf::Vector2f t = rv - velAlongNormal * normal;
        float tLen = length(t);
        if (tLen > 1e-6f) t /= tLen; else t = sf::Vector2f(0,0);

        float velAlongT = dot(rv, t);
        float jt = -velAlongT;
        jt /= invMassSum;
        jt /= static_cast<float>(m.contactCount);

        // Coulomb clamp
        sf::Vector2f frictionImpulse;
        if (std::fabs(jt) <= j * mu) {
            frictionImpulse = jt * t;
        } else {
            frictionImpulse = -j * mu * t;
        }

        if (A.invMass > 0) {
            A.vel -= frictionImpulse * A.invMass;
            A.angVel -= A.invInertia * cross(ra, frictionImpulse);
        }
        if (B.invMass > 0) {
            B.vel += frictionImpulse * B.invMass;
            B.angVel += B.invInertia * cross(rb, frictionImpulse);
        }
    }
}

// ---------- Demo ----------
int main() {
    sf::RenderWindow window(sf::VideoMode({1280u, 720u}), "Rotational Physics + SAT");
    window.setFramerateLimit(60);

    // World
    const sf::Vector2f gravity(0.f, 800.f);
    const float linearDampingPerSec = 0.98f;  // keep 98%/sec
    const float angularDampingPerSec = 0.98f;

    // Bodies
    RigidBody box;
    box.makeBox({400.f, 200.f}, {200.f, 40.f}, /*density*/0.0025f, /*static*/false, sf::Color::White);
    box.restitution = 0.2f; box.friction = 0.8f;

    RigidBody ground;
    ground.makeBox({640.f, 650.f}, {900.f, 30.f}, /*density*/1.f, /*static*/true, sf::Color(180,180,220));

    std::vector<RigidBody*> bodies = { &box, &ground };

    sf::Clock clock;
    while (window.isOpen()) {
        // Events (SFML 3 style)
        while (auto e = window.pollEvent()) {
            if (e->is<sf::Event::Closed>()) window.close();
            // Left click: apply an off-center impulse at box's top-right corner to make it spin
            if (auto me = e->getIf<sf::Event::MouseButtonPressed>()) {
                if (me->button == sf::Mouse::Button::Left) {
                    // top-right corner in world
                    float c = std::cos(box.angle), s = std::sin(box.angle);
                    sf::Vector2f cornerLocal = { box.halfExtents.x, -box.halfExtents.y };
                    sf::Vector2f cornerWorld = box.pos + sf::Vector2f(c*cornerLocal.x - s*cornerLocal.y,
                                                                      s*cornerLocal.x + c*cornerLocal.y);
                    // impulse direction towards mouse
                    sf::Vector2f mouse = (sf::Vector2f)sf::Mouse::getPosition(window);
                    sf::Vector2f dir = normalize(mouse - cornerWorld);
                    box.applyImpulseAtPoint(300.f * dir, cornerWorld);
                }
            }
        }

        float dt = clock.restart().asSeconds();

        // Integrate motion
        for (auto* rb : bodies) rb->integrate(gravity, linearDampingPerSec, angularDampingPerSec, dt);

        // Collision detect/resolve
        // (only box vs ground here; extend with nested loops for many bodies)
        Manifold m = collideOBB(box, ground);
        if (m.colliding) {
            resolveCollision(box, ground, m);
            // sync graphics (pos/angle might have been corrected by solver)
            box.drawable.setPosition(box.pos);
            box.drawable.setRotation(sf::degrees(box.angle * 180.f / 3.1415926535f));
        }

        // Draw
        window.clear(sf::Color::Black);
        window.draw(ground.drawable);
        window.draw(box.drawable);
        // Debug draw contacts
        // (small circles)
        for (int i=0;i<m.contactCount;++i) {
            sf::CircleShape c(3.f);
            c.setOrigin(sf::Vector2f(3.f,3.f));
            c.setPosition(m.contacts[i]);
            c.setFillColor(sf::Color::Red);
            window.draw(c);
        }
        window.display();
    }
}

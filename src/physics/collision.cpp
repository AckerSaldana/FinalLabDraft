#include "physics/collision.hpp"

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include <algorithm>
#include <cmath>

namespace finalLab::physics {

namespace {

float sphere_world_radius(const RigidBody& b) {
    return b.shape.sphere.radius * std::max({b.effective_scale.x, b.effective_scale.y, b.effective_scale.z});
}

glm::mat3 world_inverse_inertia(const RigidBody& b) {
    if (b.inverse_mass <= 0.0f) return glm::mat3(0.0f);
    glm::mat3 R = glm::mat3_cast(b.orientation);
    glm::mat3 D(0.0f);
    D[0][0] = b.inverse_inertia_body.x;
    D[1][1] = b.inverse_inertia_body.y;
    D[2][2] = b.inverse_inertia_body.z;
    return R * D * glm::transpose(R);
}

struct CapsuleWorld { glm::vec3 e_top; glm::vec3 e_bot; float radius; };

CapsuleWorld capsule_world(const RigidBody& cap) {
    glm::vec3 axis = cap.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
    float r     = cap.shape.capsule.radius * std::max(cap.effective_scale.x, cap.effective_scale.z);
    float halfH = cap.shape.capsule.height * cap.effective_scale.y * 0.5f;
    return { cap.position + axis * halfH, cap.position - axis * halfH, r };
}

void closest_point_on_segment(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, glm::vec3& out) {
    glm::vec3 ab = b - a;
    float ab_len2 = glm::dot(ab, ab);
    if (ab_len2 < 1e-12f) { out = a; return; }
    float t = glm::clamp(glm::dot(p - a, ab) / ab_len2, 0.0f, 1.0f);
    out = a + t * ab;
}

void closest_points_segments(const glm::vec3& p1, const glm::vec3& q1,
                             const glm::vec3& p2, const glm::vec3& q2,
                             glm::vec3& c1, glm::vec3& c2) {
    constexpr float EPS = 1e-10f;
    glm::vec3 d1 = q1 - p1;
    glm::vec3 d2 = q2 - p2;
    glm::vec3 r  = p1 - p2;
    float a = glm::dot(d1, d1);
    float e = glm::dot(d2, d2);
    float f = glm::dot(d2, r);
    float s = 0.0f, t = 0.0f;

    if (a <= EPS && e <= EPS) { c1 = p1; c2 = p2; return; }
    if (a <= EPS) {
        s = 0.0f;
        t = glm::clamp(f / e, 0.0f, 1.0f);
    } else {
        float c_ = glm::dot(d1, r);
        if (e <= EPS) {
            t = 0.0f;
            s = glm::clamp(-c_ / a, 0.0f, 1.0f);
        } else {
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b;
            s = (denom != 0.0f) ? glm::clamp((b * f - c_ * e) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = glm::clamp(-c_ / a, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = glm::clamp((b - c_) / a, 0.0f, 1.0f); }
        }
    }
    c1 = p1 + s * d1;
    c2 = p2 + t * d2;
}

// ───────────────────────── SOLID-SOLID pairs ─────────────────────────

void emit_sphere_sphere(const RigidBody& a, const RigidBody& b, int ai, int bi, std::vector<Contact>& out) {
    glm::vec3 d = b.position - a.position;
    float dist2 = glm::dot(d, d);
    float ra = sphere_world_radius(a);
    float rb = sphere_world_radius(b);
    float sum = ra + rb;
    if (dist2 >= sum * sum) return;
    float dist = std::sqrt(dist2);
    glm::vec3 n = (dist > 1e-6f) ? d / dist : glm::vec3(0.0f, 1.0f, 0.0f);
    Contact c;
    c.body_a = ai; c.body_b = bi;
    c.normal = n;
    c.penetration = sum - dist;
    c.point = a.position + n * ra;
    out.push_back(c);
}

void emit_sphere_plane(const RigidBody& sph, const RigidBody& pl, int si, int pi, std::vector<Contact>& out) {
    glm::vec3 n_world = glm::normalize(pl.orientation * pl.shape.plane.normal);
    float d = glm::dot(n_world, sph.position - pl.position);
    float r = sphere_world_radius(sph);
    if (d >= r) return;
    Contact c;
    c.body_a = pi; c.body_b = si;
    c.normal = n_world;
    c.penetration = r - d;
    c.point = sph.position - n_world * d;
    out.push_back(c);
}

void emit_sphere_cuboid(const RigidBody& sph, const RigidBody& cub, int si, int ci, std::vector<Contact>& out) {
    glm::quat inv_q = glm::inverse(cub.orientation);
    glm::vec3 c_local = inv_q * (sph.position - cub.position);
    glm::vec3 he = (cub.shape.cuboid.size * cub.effective_scale) * 0.5f;

    glm::vec3 closest_local = glm::clamp(c_local, -he, he);
    glm::vec3 delta_local   = c_local - closest_local;
    float dist2 = glm::dot(delta_local, delta_local);
    float r = sphere_world_radius(sph);
    if (dist2 >= r * r) return;

    glm::vec3 n_local;
    float dist;
    if (dist2 > 1e-12f) {
        dist = std::sqrt(dist2);
        n_local = delta_local / dist;
    } else {
        glm::vec3 face_dist = he - glm::abs(c_local);
        if (face_dist.x < face_dist.y && face_dist.x < face_dist.z) {
            n_local = { (c_local.x >= 0.0f ? 1.0f : -1.0f), 0.0f, 0.0f };
            dist = -face_dist.x;
        } else if (face_dist.y < face_dist.z) {
            n_local = { 0.0f, (c_local.y >= 0.0f ? 1.0f : -1.0f), 0.0f };
            dist = -face_dist.y;
        } else {
            n_local = { 0.0f, 0.0f, (c_local.z >= 0.0f ? 1.0f : -1.0f) };
            dist = -face_dist.z;
        }
    }

    Contact c;
    c.body_a = ci; c.body_b = si;
    c.normal = cub.orientation * n_local;
    c.penetration = r - dist;
    c.point = cub.position + cub.orientation * closest_local;
    out.push_back(c);
}

void emit_sphere_cylinder(const RigidBody& sph, const RigidBody& cyl, int si, int ci, std::vector<Contact>& out) {
    glm::quat inv_q = glm::inverse(cyl.orientation);
    glm::vec3 c_local = inv_q * (sph.position - cyl.position);

    float R     = cyl.shape.cylinder.radius * std::max(cyl.effective_scale.x, cyl.effective_scale.z);
    float halfH = cyl.shape.cylinder.height * cyl.effective_scale.y * 0.5f;
    float r     = sphere_world_radius(sph);

    glm::vec3 closest_local;
    closest_local.y = glm::clamp(c_local.y, -halfH, halfH);
    float radial = std::sqrt(c_local.x * c_local.x + c_local.z * c_local.z);
    if (radial > R) {
        float k = R / radial;
        closest_local.x = c_local.x * k;
        closest_local.z = c_local.z * k;
    } else {
        closest_local.x = c_local.x;
        closest_local.z = c_local.z;
    }

    glm::vec3 delta = c_local - closest_local;
    float dist2 = glm::dot(delta, delta);
    if (dist2 >= r * r) return;

    glm::vec3 n_local;
    float dist;
    if (dist2 > 1e-12f) {
        dist = std::sqrt(dist2);
        n_local = delta / dist;
    } else {
        float radial_exit = R - radial;
        float top_exit    = halfH - c_local.y;
        float bot_exit    = c_local.y + halfH;
        float min_exit = std::min({ radial_exit, top_exit, bot_exit });
        if (min_exit == radial_exit) {
            if (radial > 1e-6f) {
                n_local = glm::vec3(c_local.x / radial, 0.0f, c_local.z / radial);
            } else {
                n_local = glm::vec3(1.0f, 0.0f, 0.0f);
            }
            dist = -radial_exit;
        } else if (min_exit == top_exit) {
            n_local = glm::vec3(0.0f, 1.0f, 0.0f);
            dist = -top_exit;
        } else {
            n_local = glm::vec3(0.0f, -1.0f, 0.0f);
            dist = -bot_exit;
        }
    }

    Contact c;
    c.body_a = ci; c.body_b = si;
    c.normal = cyl.orientation * n_local;
    c.penetration = r - dist;
    c.point = cyl.position + cyl.orientation * closest_local;
    out.push_back(c);
}

void emit_sphere_capsule(const RigidBody& sph, const RigidBody& cap, int si, int ci, std::vector<Contact>& out) {
    CapsuleWorld cw = capsule_world(cap);
    glm::vec3 closest;
    closest_point_on_segment(sph.position, cw.e_bot, cw.e_top, closest);

    glm::vec3 d = sph.position - closest;
    float dist2 = glm::dot(d, d);
    float r_sph = sphere_world_radius(sph);
    float sum = r_sph + cw.radius;
    if (dist2 >= sum * sum) return;

    float dist = std::sqrt(dist2);
    glm::vec3 n = (dist > 1e-6f) ? d / dist : glm::vec3(0.0f, 1.0f, 0.0f);
    Contact c;
    c.body_a = ci; c.body_b = si;
    c.normal = n;
    c.penetration = sum - dist;
    c.point = closest + n * cw.radius;
    out.push_back(c);
}

void emit_capsule_capsule(const RigidBody& a, const RigidBody& b, int ai, int bi, std::vector<Contact>& out) {
    CapsuleWorld ca = capsule_world(a);
    CapsuleWorld cb = capsule_world(b);

    glm::vec3 c1, c2;
    closest_points_segments(ca.e_bot, ca.e_top, cb.e_bot, cb.e_top, c1, c2);

    glm::vec3 d = c2 - c1;
    float dist2 = glm::dot(d, d);
    float sum = ca.radius + cb.radius;
    if (dist2 >= sum * sum) return;

    float dist = std::sqrt(dist2);
    glm::vec3 n = (dist > 1e-6f) ? d / dist : glm::vec3(0.0f, 1.0f, 0.0f);
    Contact c;
    c.body_a = ai; c.body_b = bi;
    c.normal = n;
    c.penetration = sum - dist;
    c.point = c1 + n * ca.radius;
    out.push_back(c);
}

void emit_cuboid_plane(const RigidBody& cub, const RigidBody& pl, int ci, int pi, std::vector<Contact>& out) {
    glm::vec3 n_world = glm::normalize(pl.orientation * pl.shape.plane.normal);
    glm::vec3 he = (cub.shape.cuboid.size * cub.effective_scale) * 0.5f;
    for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glm::vec3 v_local(sx * he.x, sy * he.y, sz * he.z);
        glm::vec3 v_world = cub.position + cub.orientation * v_local;
        float d = glm::dot(n_world, v_world - pl.position);
        if (d >= 0.0f) continue;
        Contact c;
        c.body_a = pi; c.body_b = ci;
        c.normal = n_world;
        c.penetration = -d;
        c.point = v_world - n_world * d;
        out.push_back(c);
    }
}

void emit_cylinder_plane(const RigidBody& cyl, const RigidBody& pl, int ci, int pi, std::vector<Contact>& out) {
    glm::vec3 n_world = glm::normalize(pl.orientation * pl.shape.plane.normal);
    glm::vec3 axis    = cyl.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
    float halfH = cyl.shape.cylinder.height * cyl.effective_scale.y * 0.5f;
    float R     = cyl.shape.cylinder.radius * std::max(cyl.effective_scale.x, cyl.effective_scale.z);

    glm::vec3 d = -n_world;
    glm::vec3 d_perp = d - glm::dot(d, axis) * axis;
    float dp_len = glm::length(d_perp);
    glm::vec3 perp;
    if (dp_len > 1e-6f) {
        perp = d_perp / dp_len;
    } else {
        glm::vec3 helper = std::abs(axis.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        perp = glm::normalize(glm::cross(axis, helper));
    }

    for (int sign = -1; sign <= 1; sign += 2) {
        glm::vec3 cap_center = cyl.position + axis * (static_cast<float>(sign) * halfH);
        glm::vec3 lowest = cap_center + perp * R;
        float depth = glm::dot(n_world, lowest - pl.position);
        if (depth >= 0.0f) continue;
        Contact c;
        c.body_a = pi; c.body_b = ci;
        c.normal = n_world;
        c.penetration = -depth;
        c.point = lowest - n_world * depth;
        out.push_back(c);
    }
}

void emit_capsule_plane(const RigidBody& cap, const RigidBody& pl, int ci, int pi, std::vector<Contact>& out) {
    CapsuleWorld cw = capsule_world(cap);
    glm::vec3 n_world = glm::normalize(pl.orientation * pl.shape.plane.normal);
    glm::vec3 endpoints[2] = { cw.e_top, cw.e_bot };
    for (const glm::vec3& e : endpoints) {
        float d = glm::dot(n_world, e - pl.position);
        if (d >= cw.radius) continue;
        Contact c;
        c.body_a = pi; c.body_b = ci;
        c.normal = n_world;
        c.penetration = cw.radius - d;
        c.point = e - n_world * d;
        out.push_back(c);
    }
}

// Reusable: emit a sphere-vs-cuboid contact at a synthetic sphere centred at
// `world_pos` with radius `radius`, against the cuboid `cub`. Used by capsule-
// cuboid and cylinder-cuboid sampling — both bodies have rotational symmetry
// around their local Y axis, so sampling a few "stand-in" spheres along that
// axis approximates the swept volume well enough for stable stacking and bounces.
void emit_synth_sphere_cuboid(const RigidBody& bodyA, int aiA, float radius, const glm::vec3& world_pos,
                              const RigidBody& cub, int ci, std::vector<Contact>& out) {
    glm::quat inv_q = glm::inverse(cub.orientation);
    glm::vec3 c_local = inv_q * (world_pos - cub.position);
    glm::vec3 he = (cub.shape.cuboid.size * cub.effective_scale) * 0.5f;

    glm::vec3 closest = glm::clamp(c_local, -he, he);
    glm::vec3 d = c_local - closest;
    float d2 = glm::dot(d, d);
    if (d2 >= radius * radius) {
        // Sphere centre may be inside the box — pick the smallest exit axis.
        if (std::abs(c_local.x) <= he.x && std::abs(c_local.y) <= he.y && std::abs(c_local.z) <= he.z) {
            float dx = he.x - std::abs(c_local.x);
            float dy = he.y - std::abs(c_local.y);
            float dz = he.z - std::abs(c_local.z);
            glm::vec3 n_local(0.0f);
            float pen;
            if (dx <= dy && dx <= dz) { n_local.x = (c_local.x >= 0 ? 1.0f : -1.0f); pen = dx + radius; closest = c_local; closest.x = n_local.x * he.x; }
            else if (dy <= dz)        { n_local.y = (c_local.y >= 0 ? 1.0f : -1.0f); pen = dy + radius; closest = c_local; closest.y = n_local.y * he.y; }
            else                      { n_local.z = (c_local.z >= 0 ? 1.0f : -1.0f); pen = dz + radius; closest = c_local; closest.z = n_local.z * he.z; }
            Contact c;
            c.body_a = aiA; c.body_b = ci;
            c.normal = -(cub.orientation * n_local);  // A→B, so push A outward from cuboid
            c.penetration = pen;
            c.point = cub.position + cub.orientation * closest;
            out.push_back(c);
        }
        return;
    }
    float dist = std::sqrt(d2);
    glm::vec3 n_local = (dist > 1e-6f) ? d / dist : glm::vec3(0, 1, 0);
    Contact c;
    c.body_a = aiA; c.body_b = ci;
    c.normal = -(cub.orientation * n_local);
    c.penetration = radius - dist;
    c.point = cub.position + cub.orientation * closest;
    out.push_back(c);
    (void)bodyA;  // bodyA reserved for future use (e.g. anchored contact point)
}

void emit_capsule_cuboid(const RigidBody& cap, const RigidBody& cub, int ai, int ci, std::vector<Contact>& out) {
    CapsuleWorld cw = capsule_world(cap);
    // 3 sample spheres: both endpoints + midpoint. Enough to catch face/edge
    // contact in typical configurations without exploding the contact count.
    glm::vec3 mid = 0.5f * (cw.e_top + cw.e_bot);
    emit_synth_sphere_cuboid(cap, ai, cw.radius, cw.e_top, cub, ci, out);
    emit_synth_sphere_cuboid(cap, ai, cw.radius, mid,      cub, ci, out);
    emit_synth_sphere_cuboid(cap, ai, cw.radius, cw.e_bot, cub, ci, out);
}

void emit_cylinder_cuboid(const RigidBody& cyl, const RigidBody& cub, int ai, int ci, std::vector<Contact>& out) {
    glm::vec3 axis = cyl.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
    float halfH = cyl.shape.cylinder.height * cyl.effective_scale.y * 0.5f;
    float r     = cyl.shape.cylinder.radius * std::max(cyl.effective_scale.x, cyl.effective_scale.z);
    // 5 spheres along the cylinder's axis. Cylinder has uniform radius along
    // its axis (unlike a capsule, which only swells at the caps), so the
    // approximation is tighter than the capsule version above.
    for (int k = -2; k <= 2; ++k) {
        glm::vec3 p = cyl.position + axis * (static_cast<float>(k) * 0.5f * halfH);
        emit_synth_sphere_cuboid(cyl, ai, r, p, cub, ci, out);
    }
}

// Cuboid-cuboid via SAT (15 candidate axes: 3 face normals each, 9 edge crosses).
// Contact generation uses the cheap-and-stable "vertices-inside-the-other-box"
// scheme: any vertex of box A penetrating box B (in B's local frame) becomes a
// contact along the SAT-min normal. This gives 4 corner contacts for stacked
// boxes, which is enough for a stable resting stack — the iterative solver +
// Baumgarte correction handle the rest. For edge-edge (a cross axis wins) we
// fall back to a single contact at the midpoint of the closest edge pair.
void emit_cuboid_cuboid(const RigidBody& a, const RigidBody& b, int ai, int bi, std::vector<Contact>& out) {
    glm::vec3 ha = (a.shape.cuboid.size * a.effective_scale) * 0.5f;
    glm::vec3 hb = (b.shape.cuboid.size * b.effective_scale) * 0.5f;

    glm::mat3 Ra = glm::mat3_cast(a.orientation);
    glm::mat3 Rb = glm::mat3_cast(b.orientation);
    glm::vec3 Au[3] = { Ra[0], Ra[1], Ra[2] };
    glm::vec3 Bu[3] = { Rb[0], Rb[1], Rb[2] };

    glm::vec3 t_world = b.position - a.position;

    // Candidate axes (15). Track the one with smallest positive overlap.
    glm::vec3 axes[15];
    int axis_count = 0;
    for (int i = 0; i < 3; ++i) axes[axis_count++] = Au[i];
    for (int i = 0; i < 3; ++i) axes[axis_count++] = Bu[i];
    bool from_cross[15] = { false, false, false, false, false, false };
    int  cross_i[15]    = { 0, 0, 0, 0, 0, 0 };
    int  cross_j[15]    = { 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm::vec3 c = glm::cross(Au[i], Bu[j]);
            from_cross[axis_count] = true;
            cross_i[axis_count]    = i;
            cross_j[axis_count]    = j;
            axes[axis_count++]     = c;
        }
    }

    float best_overlap = 1e30f;
    glm::vec3 best_axis(0.0f);
    bool best_from_cross = false;
    int  best_ci = 0, best_cj = 0;

    for (int k = 0; k < axis_count; ++k) {
        glm::vec3 ax = axes[k];
        float len2 = glm::dot(ax, ax);
        if (len2 < 1e-8f) continue;        // parallel edges → degenerate axis, skip
        ax /= std::sqrt(len2);

        float ra = ha.x * std::fabs(glm::dot(Au[0], ax))
                 + ha.y * std::fabs(glm::dot(Au[1], ax))
                 + ha.z * std::fabs(glm::dot(Au[2], ax));
        float rb = hb.x * std::fabs(glm::dot(Bu[0], ax))
                 + hb.y * std::fabs(glm::dot(Bu[1], ax))
                 + hb.z * std::fabs(glm::dot(Bu[2], ax));
        float dist = std::fabs(glm::dot(t_world, ax));
        float overlap = (ra + rb) - dist;
        if (overlap <= 0.0f) return;       // separating axis found → no contact

        if (overlap < best_overlap) {
            best_overlap    = overlap;
            best_axis       = ax;
            best_from_cross = from_cross[k];
            best_ci         = cross_i[k];
            best_cj         = cross_j[k];
        }
    }

    // Convention: contact normal points from A to B.
    glm::vec3 n = best_axis;
    if (glm::dot(n, t_world) < 0.0f) n = -n;

    if (!best_from_cross) {
        // Face contact: collect ALL penetrating vertices (from both boxes) and
        // emit ONE single contact at their centroid with the SAT penetration.
        // The previous design emitted up to 8 contacts (4 per box face); each
        // contact applied position correction with the same penetration value,
        // compounding 8× per frame — a stacked cube pushed the cube below it
        // through the floor faster than the floor contact could correct it.
        glm::quat inv_a = glm::inverse(a.orientation);
        glm::quat inv_b = glm::inverse(b.orientation);

        glm::vec3 sum_pt(0.0f);
        int n_pts = 0;
        for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
            glm::vec3 v_world = a.position + a.orientation * glm::vec3(sx*ha.x, sy*ha.y, sz*ha.z);
            glm::vec3 v_local_b = inv_b * (v_world - b.position);
            if (std::fabs(v_local_b.x) <= hb.x &&
                std::fabs(v_local_b.y) <= hb.y &&
                std::fabs(v_local_b.z) <= hb.z) {
                sum_pt += v_world;
                ++n_pts;
            }
        }
        for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
            glm::vec3 v_world = b.position + b.orientation * glm::vec3(sx*hb.x, sy*hb.y, sz*hb.z);
            glm::vec3 v_local_a = inv_a * (v_world - a.position);
            if (std::fabs(v_local_a.x) <= ha.x &&
                std::fabs(v_local_a.y) <= ha.y &&
                std::fabs(v_local_a.z) <= ha.z) {
                sum_pt += v_world;
                ++n_pts;
            }
        }

        Contact c;
        c.body_a = ai; c.body_b = bi;
        c.normal = n;
        c.penetration = best_overlap;
        c.point = (n_pts > 0) ? (sum_pt / static_cast<float>(n_pts))
                              : (a.position + 0.5f * t_world);
        out.push_back(c);
        return;
    }

    // Edge-edge: pick the two edges whose cross is best_axis (best_ci, best_cj),
    // find the segment-segment closest points, and place one contact at their midpoint.
    glm::vec3 ea = Au[best_ci];
    glm::vec3 eb = Bu[best_cj];

    // Pick the corner of A that lies on the "deepest" edge along n: take the
    // sign that maximises dot(corner - a.pos, n) — that corner is on the side
    // of A pointing toward B.
    auto pick_corner = [&](const RigidBody& body, const glm::vec3 he, const glm::vec3* axes3,
                           const glm::vec3& wantDir, int skip_axis) -> glm::vec3 {
        glm::vec3 c(0.0f);
        for (int k = 0; k < 3; ++k) {
            if (k == skip_axis) continue;
            float s = (glm::dot(axes3[k], wantDir) >= 0.0f) ? 1.0f : -1.0f;
            c += axes3[k] * (s * he[k]);
        }
        return body.position + c;
    };
    glm::vec3 pA = pick_corner(a, ha, Au, n,  best_ci);
    glm::vec3 pB = pick_corner(b, hb, Bu, -n, best_cj);
    glm::vec3 qA = pA + ea * ha[best_ci];
    glm::vec3 pA2 = pA - ea * ha[best_ci];
    glm::vec3 qB = pB + eb * hb[best_cj];
    glm::vec3 pB2 = pB - eb * hb[best_cj];

    glm::vec3 c1, c2;
    closest_points_segments(pA2, qA, pB2, qB, c1, c2);

    Contact c;
    c.body_a = ai; c.body_b = bi;
    c.normal = n;
    c.penetration = best_overlap;
    c.point = 0.5f * (c1 + c2);
    out.push_back(c);
}

// ─────────────────────── SOLID-CONTAINER pairs ───────────────────────

void emit_sphere_in_sphere_container(const RigidBody& sph, const RigidBody& cont, int si, int ci, std::vector<Contact>& out) {
    glm::vec3 d = sph.position - cont.position;
    float dist = glm::length(d);
    float r_sph  = sphere_world_radius(sph);
    float r_cont = sphere_world_radius(cont);
    // Fully inside (sphere doesn't touch the wall) → no contact.
    if (dist + r_sph <= r_cont) return;
    // Otherwise — sphere is touching or has even crossed the wall — emit a
    // contact pulling it inward. We do NOT early-return when dist >= r_cont,
    // because that would let escaped bodies fly off forever; instead we still
    // generate an inward-pointing impulse so the resolver brings them back.

    glm::vec3 outward = (dist > 1e-6f) ? d / dist : glm::vec3(0.0f, 1.0f, 0.0f);
    Contact c;
    c.body_a = ci; c.body_b = si;
    c.normal = -outward;
    c.penetration = (dist + r_sph) - r_cont;
    c.point = cont.position + outward * r_cont;
    out.push_back(c);
}

void emit_sphere_in_cuboid_container(const RigidBody& sph, const RigidBody& cont, int si, int ci, std::vector<Contact>& out) {
    glm::quat inv_q = glm::inverse(cont.orientation);
    glm::vec3 c_local = inv_q * (sph.position - cont.position);
    glm::vec3 he = (cont.shape.cuboid.size * cont.effective_scale) * 0.5f;
    float r = sphere_world_radius(sph);

    // No "sphere fully outside" early-return — the per-axis test below still
    // produces inward-pulling contacts for spheres that have escaped, so the
    // solver brings them back instead of losing them.

    for (int axis = 0; axis < 3; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            float face = static_cast<float>(sign) * he[axis];
            float comp = c_local[axis];
            float pen  = (sign > 0) ? (comp + r - face) : (face - (comp - r));
            if (pen <= 0.0f) continue;
            glm::vec3 n_local(0.0f);
            n_local[axis] = -static_cast<float>(sign);
            glm::vec3 cp_local = c_local;
            cp_local[axis] = face;
            Contact c;
            c.body_a = ci; c.body_b = si;
            c.normal = cont.orientation * n_local;
            c.penetration = pen;
            c.point = cont.position + cont.orientation * cp_local;
            out.push_back(c);
        }
    }
}

void emit_sphere_in_cylinder_container(const RigidBody& sph, const RigidBody& cont, int si, int ci, std::vector<Contact>& out) {
    glm::quat inv_q = glm::inverse(cont.orientation);
    glm::vec3 c_local = inv_q * (sph.position - cont.position);

    float R     = cont.shape.cylinder.radius * std::max(cont.effective_scale.x, cont.effective_scale.z);
    float H     = cont.shape.cylinder.height * cont.effective_scale.y;
    float halfH = H * 0.5f;
    float r     = sphere_world_radius(sph);

    float radial = std::sqrt(c_local.x * c_local.x + c_local.z * c_local.z);

    // No early-return on "sphere fully outside" — the side/cap tests below
    // emit inward-pulling contacts for any sphere that pokes past a wall.

    if (radial + r > R) {
        glm::vec3 n_local;
        glm::vec3 cp_local = c_local;
        if (radial > 1e-6f) {
            float inv = 1.0f / radial;
            n_local = { -c_local.x * inv, 0.0f, -c_local.z * inv };
            cp_local.x = c_local.x * inv * R;
            cp_local.z = c_local.z * inv * R;
        } else {
            n_local = { 0.0f, 0.0f, -1.0f };
            cp_local.x = 0.0f; cp_local.z = R;
        }
        Contact c;
        c.body_a = ci; c.body_b = si;
        c.normal = cont.orientation * n_local;
        c.penetration = (radial + r) - R;
        c.point = cont.position + cont.orientation * cp_local;
        out.push_back(c);
    }
    if (c_local.y + r > halfH) {
        Contact c;
        c.body_a = ci; c.body_b = si;
        c.normal = cont.orientation * glm::vec3(0.0f, -1.0f, 0.0f);
        c.penetration = (c_local.y + r) - halfH;
        glm::vec3 cp_local = c_local; cp_local.y = halfH;
        c.point = cont.position + cont.orientation * cp_local;
        out.push_back(c);
    }
    if (c_local.y - r < -halfH) {
        Contact c;
        c.body_a = ci; c.body_b = si;
        c.normal = cont.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
        c.penetration = -halfH - (c_local.y - r);
        glm::vec3 cp_local = c_local; cp_local.y = -halfH;
        c.point = cont.position + cont.orientation * cp_local;
        out.push_back(c);
    }
}

// Cuboid inside a cuboid container: scan the 8 vertices of the inner box,
// then emit AT MOST one contact per (axis, sign) face of the container — using
// the deepest penetration found and the centroid of the penetrating vertices
// as the contact point. Emitting one contact per vertex (the previous design)
// caused position correction to compound 4× per face, jolting cubes against
// walls and creating pseudo-static "floating" states for spinning bodies.
void emit_cuboid_in_cuboid_container(const RigidBody& cub, const RigidBody& cont, int ai, int ci, std::vector<Contact>& out) {
    glm::quat inv_q = glm::inverse(cont.orientation);
    glm::vec3 he_in  = (cub.shape.cuboid.size  * cub.effective_scale)  * 0.5f;
    glm::vec3 he_out = (cont.shape.cuboid.size * cont.effective_scale) * 0.5f;

    // Per-face accumulator: indices [axis][sign] where sign 0=+axis, 1=-axis.
    struct FaceAcc { float deepest = 0.0f; glm::vec3 sum_pt{0.0f}; int n = 0; };
    FaceAcc face[3][2];

    for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glm::vec3 v_world = cub.position + cub.orientation * glm::vec3(sx*he_in.x, sy*he_in.y, sz*he_in.z);
        glm::vec3 v_local = inv_q * (v_world - cont.position);

        for (int axis = 0; axis < 3; ++axis) {
            float comp = v_local[axis];
            if (comp > he_out[axis]) {
                glm::vec3 cp_local = v_local; cp_local[axis] = he_out[axis];
                FaceAcc& f = face[axis][0];
                f.deepest = std::max(f.deepest, comp - he_out[axis]);
                f.sum_pt += cont.position + cont.orientation * cp_local;
                ++f.n;
            } else if (comp < -he_out[axis]) {
                glm::vec3 cp_local = v_local; cp_local[axis] = -he_out[axis];
                FaceAcc& f = face[axis][1];
                f.deepest = std::max(f.deepest, -he_out[axis] - comp);
                f.sum_pt += cont.position + cont.orientation * cp_local;
                ++f.n;
            }
        }
    }

    for (int axis = 0; axis < 3; ++axis) {
        for (int s = 0; s < 2; ++s) {
            const FaceAcc& f = face[axis][s];
            if (f.n == 0) continue;
            glm::vec3 n_local(0.0f);
            n_local[axis] = (s == 0) ? -1.0f : 1.0f;        // inward
            Contact c;
            c.body_a = ci; c.body_b = ai;
            c.normal      = cont.orientation * n_local;
            c.penetration = f.deepest;
            c.point       = f.sum_pt / static_cast<float>(f.n);
            out.push_back(c);
        }
    }
}

// Cuboid inside a sphere container: each vertex of the cuboid that strays
// outside the sphere generates an inward radial contact.
void emit_cuboid_in_sphere_container(const RigidBody& cub, const RigidBody& cont, int ai, int ci, std::vector<Contact>& out) {
    glm::vec3 he = (cub.shape.cuboid.size * cub.effective_scale) * 0.5f;
    float r_cont = sphere_world_radius(cont);

    for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glm::vec3 v_world = cub.position + cub.orientation * glm::vec3(sx*he.x, sy*he.y, sz*he.z);
        glm::vec3 d = v_world - cont.position;
        float dist = glm::length(d);
        if (dist <= r_cont) continue;
        glm::vec3 outward = (dist > 1e-6f) ? d / dist : glm::vec3(0.0f, 1.0f, 0.0f);
        Contact c;
        c.body_a = ci; c.body_b = ai;
        c.normal = -outward;
        c.penetration = dist - r_cont;
        c.point = cont.position + outward * r_cont;
        out.push_back(c);
    }
}

// ────────────────────────── Pair dispatcher ──────────────────────────

void dispatch(const RigidBody& a_in, const RigidBody& b_in, int ai_in, int bi_in, std::vector<Contact>& out) {
    if (a_in.inverse_mass <= 0.0f && b_in.inverse_mass <= 0.0f) return;

    const RigidBody* a = &a_in;
    const RigidBody* b = &b_in;
    int ai = ai_in, bi = bi_in;

    if (b->collision_type == scene::CollisionType::Solid &&
        a->collision_type == scene::CollisionType::Container) {
        std::swap(a, b); std::swap(ai, bi);
    }
    if (a->collision_type == scene::CollisionType::Container) return;

    using SK = scene::ShapeKind;

    if (b->collision_type == scene::CollisionType::Solid) {
        if (static_cast<int>(b->shape.kind) < static_cast<int>(a->shape.kind)) {
            std::swap(a, b); std::swap(ai, bi);
        }
        switch (a->shape.kind) {
            case SK::Sphere:
                switch (b->shape.kind) {
                    case SK::Sphere:   emit_sphere_sphere  (*a, *b, ai, bi, out); break;
                    case SK::Cuboid:   emit_sphere_cuboid  (*a, *b, ai, bi, out); break;
                    case SK::Cylinder: emit_sphere_cylinder(*a, *b, ai, bi, out); break;
                    case SK::Capsule:  emit_sphere_capsule (*a, *b, ai, bi, out); break;
                    case SK::Plane:    emit_sphere_plane   (*a, *b, ai, bi, out); break;
                }
                break;
            case SK::Cuboid:
                switch (b->shape.kind) {
                    case SK::Cuboid:   emit_cuboid_cuboid(*a, *b, ai, bi, out); break;
                    case SK::Cylinder: emit_cylinder_cuboid(*b, *a, bi, ai, out); break;  // a=cub, b=cyl
                    case SK::Capsule:  emit_capsule_cuboid (*b, *a, bi, ai, out); break;  // a=cub, b=cap
                    case SK::Plane:    emit_cuboid_plane (*a, *b, ai, bi, out); break;
                    default: break;
                }
                break;
            case SK::Cylinder:
                switch (b->shape.kind) {
                    case SK::Plane: emit_cylinder_plane(*a, *b, ai, bi, out); break;
                    default: break;
                }
                break;
            case SK::Capsule:
                switch (b->shape.kind) {
                    case SK::Capsule: emit_capsule_capsule(*a, *b, ai, bi, out); break;
                    case SK::Plane:   emit_capsule_plane  (*a, *b, ai, bi, out); break;
                    default: break;
                }
                break;
            default: break;
        }
    } else {
        // a = solid, b = container.
        switch (a->shape.kind) {
            case SK::Sphere:
                switch (b->shape.kind) {
                    case SK::Sphere:   emit_sphere_in_sphere_container  (*a, *b, ai, bi, out); break;
                    case SK::Cuboid:   emit_sphere_in_cuboid_container  (*a, *b, ai, bi, out); break;
                    case SK::Cylinder: emit_sphere_in_cylinder_container(*a, *b, ai, bi, out); break;
                    default: break;
                }
                break;
            case SK::Cuboid:
                switch (b->shape.kind) {
                    case SK::Cuboid: emit_cuboid_in_cuboid_container(*a, *b, ai, bi, out); break;
                    case SK::Sphere: emit_cuboid_in_sphere_container(*a, *b, ai, bi, out); break;
                    default: break;
                }
                break;
            default: break;   // capsule/cylinder/plane container variants TBD
        }
    }
}

} // namespace

void detect_contacts(const std::vector<RigidBody>& bodies, std::vector<Contact>& out) {
    out.clear();
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            dispatch(bodies[i], bodies[j], static_cast<int>(i), static_cast<int>(j), out);
        }
    }
}

void resolve_contact(Contact& c, std::vector<RigidBody>& bodies,
                     const ResolvedInteraction& mat, float position_percent, float slop,
                     int my_peer_id) {
    RigidBody& a = bodies[c.body_a];
    RigidBody& b = bodies[c.body_b];

    bool a_mine = (my_peer_id == 0) || (a.owner_peer_id == 0) || (a.owner_peer_id == my_peer_id);
    bool b_mine = (my_peer_id == 0) || (b.owner_peer_id == 0) || (b.owner_peer_id == my_peer_id);
    if (!a_mine && !b_mine) return;

    glm::vec3 ra = c.point - a.position;
    glm::vec3 rb = c.point - b.position;

    glm::mat3 Ia_inv = world_inverse_inertia(a);
    glm::mat3 Ib_inv = world_inverse_inertia(b);

    glm::vec3 va = a.linear_velocity + glm::cross(a.angular_velocity_rad, ra);
    glm::vec3 vb = b.linear_velocity + glm::cross(b.angular_velocity_rad, rb);
    glm::vec3 v_rel = vb - va;
    float v_n = glm::dot(v_rel, c.normal);

    // Apply normal impulse + friction ONLY when the bodies are approaching
    // (v_n <= 0). When already separating we still want position correction
    // below to run, so an escaped body that's currently moving inward still
    // gets dragged the rest of the way back to the container surface.
    bool approaching = (v_n <= 0.0f);

    float j = 0.0f;
    if (approaching) {
        glm::vec3 ra_x_n = glm::cross(ra, c.normal);
        glm::vec3 rb_x_n = glm::cross(rb, c.normal);
        float inv_m_eff = a.inverse_mass + b.inverse_mass
                        + glm::dot(c.normal, glm::cross(Ia_inv * ra_x_n, ra))
                        + glm::dot(c.normal, glm::cross(Ib_inv * rb_x_n, rb));
        if (inv_m_eff > 0.0f) {
            j = -(1.0f + mat.restitution) * v_n / inv_m_eff;
            glm::vec3 J = j * c.normal;

            if (a_mine) {
                a.linear_velocity      -= J * a.inverse_mass;
                a.angular_velocity_rad -= Ia_inv * glm::cross(ra, J);
            }
            if (b_mine) {
                b.linear_velocity      += J * b.inverse_mass;
                b.angular_velocity_rad += Ib_inv * glm::cross(rb, J);
            }

            glm::vec3 va2 = a.linear_velocity + glm::cross(a.angular_velocity_rad, ra);
            glm::vec3 vb2 = b.linear_velocity + glm::cross(b.angular_velocity_rad, rb);
            glm::vec3 v_rel2 = vb2 - va2;
            glm::vec3 v_t = v_rel2 - glm::dot(v_rel2, c.normal) * c.normal;
            float v_t_len = glm::length(v_t);
            if (v_t_len > 1e-6f) {
                glm::vec3 t = v_t / v_t_len;
                glm::vec3 ra_x_t = glm::cross(ra, t);
                glm::vec3 rb_x_t = glm::cross(rb, t);
                float inv_m_t = a.inverse_mass + b.inverse_mass
                              + glm::dot(t, glm::cross(Ia_inv * ra_x_t, ra))
                              + glm::dot(t, glm::cross(Ib_inv * rb_x_t, rb));
                if (inv_m_t > 0.0f) {
                    float j_t   = -v_t_len / inv_m_t;
                    float limit = mat.dynamic_friction * std::fabs(j);
                    j_t = std::clamp(j_t, -limit, limit);
                    glm::vec3 J_t = j_t * t;
                    if (a_mine) {
                        a.linear_velocity      -= J_t * a.inverse_mass;
                        a.angular_velocity_rad -= Ia_inv * glm::cross(ra, J_t);
                    }
                    if (b_mine) {
                        b.linear_velocity      += J_t * b.inverse_mass;
                        b.angular_velocity_rad += Ib_inv * glm::cross(rb, J_t);
                    }
                }
            }
        }
    }

    // Position correction (Baumgarte) — runs unconditionally so an escaped
    // body that has already been kicked back inward still gets dragged out of
    // its overshoot. Without this, a fast-moving boid that punched through the
    // container wall could remain outside until it eventually re-approaches.
    float inv_a = a_mine ? a.inverse_mass : 0.0f;
    float inv_b = b_mine ? b.inverse_mass : 0.0f;
    float inv_m_total = inv_a + inv_b;
    if (inv_m_total > 0.0f) {
        float corr_mag = std::max(c.penetration - slop, 0.0f) * position_percent / inv_m_total;
        glm::vec3 corr = corr_mag * c.normal;
        if (a_mine) a.position -= corr * a.inverse_mass;
        if (b_mine) b.position += corr * b.inverse_mass;
    }
}

} // namespace finalLab::physics

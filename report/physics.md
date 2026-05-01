# Motion physics, collision detection, and collision response

## Rigid-body state

Each `RigidBody` stores world-space `position`, `orientation` (quaternion `w,x,y,z`), `linear_velocity`, `angular_velocity_rad`, `inverse_mass`, the body-frame inverse inertia diagonal (`inverse_inertia_body`), and metadata (shape, scale, material, collision type, owner peer). The body-frame inertia is diagonal because every shape (sphere, cuboid, cylinder, capsule, plane) is canonically aligned to its local axes, so principal moments are stored as a `vec3` and the world tensor is reconstructed on demand as `R · diag(1/Iₓ, 1/Iᵧ, 1/Iᵤ) · Rᵀ` where `R = mat3_cast(orientation)`.

Mass is `density × volume(scale)` computed at scene load — `volume` for sphere = `(4/3)πr³`, cuboid = `wxhxd`, cylinder = `πr²h`, capsule = `πr²h + (4/3)πr³`, plane = `0`. Inertia tensors are the textbook ones (e.g. cuboid `Iₓₓ = (1/12)m(h² + d²)`); the capsule is approximated as a cylinder of full extent for simplicity.

## Integration

The sim thread runs at fixed Hz (default 240, ImGui-controlled up to 2 kHz). Each step:

1. **Apply pending control commands** drained under `control_mutex_` (gravity, paused, scene swap, reset, solver iterations, flock params, etc.).
2. **Boid steering** for owned boids (Reynolds — see below), accumulating into `linear_velocity` and clamping to `max_speed`.
3. **Linear + angular integration** for owned simulated bodies — semi-implicit Euler:

   ```
   v += g · dt                  (if gravity_on)
   p += v · dt
   ω_q   = quat(0, ωₓ, ωᵧ, ω_z)
   q    += ½ · dt · (ω_q · q)
   q     = normalize(q)
   ```

   Non-owned bodies (Phase 6+) extrapolate `p += v · dt` using the last network-supplied velocity and then blend toward the network target via `1 − exp(−rate · dt)` for position and `slerp` for orientation (Phase 8 drift correction).

4. **Animated bodies** (Phase 3) sample their waypoint list at `total_time_`. The sampler honours `STOP / LOOP / REVERSE` and `LINEAR / SMOOTHSTEP` per the spec, blending position with `mix` and orientation with `slerp`. After sampling, `linear_velocity` and `angular_velocity_rad` are recomputed by finite-differencing pose against the previous step. They have `inverse_mass = 0` so the resolver applies no impulse to them — but the kinematic velocity feeds into `v_rel` so simulated bodies bounce off them correctly. Momentum is therefore transferred *from* the moving paddle *to* the simulated body and not the other way, exactly as the spec requires.

5. **Collision detection + iterative resolve** (`solver_iterations`, default 6).

## Collision detection

`detect_contacts(bodies, out)` is an O(n²) broadphase over all bodies. The dispatcher orders the pair so that container bodies come second and SOLID-SOLID pairs are sorted by `ShapeKind` ordinal, then routes to a per-pair tester. Implemented:

- **SOLID-SOLID**: sphere-sphere, sphere-plane, sphere-cuboid (oriented; sphere-centre clamped into the box's local extents), sphere-cylinder (clamp y-component, project xz onto the disk; degenerate "centre inside" case picks the smallest exit axis), sphere-capsule (closest point on segment then sphere-sphere), capsule-capsule (closest points between two segments via the standard line-line algorithm with parallel-segment fallback), cuboid-plane (8 vertices each tested individually — generates up to 8 contacts so resting boxes stack stably with 4 corner contacts), cylinder-plane (lowest perimeter point of each end-cap), capsule-plane (both endpoint spheres).

- **SOLID-CONTAINER** (inverted, "inside is empty, outside is solid"): sphere-in-sphere container (penetration when `dist + r_sphere > r_container`, normal points inward), sphere-in-cuboid container (per-axis: up to 6 contacts so corner cases are solved iteratively), sphere-in-cylinder container (side wall radial test, top + bottom caps; up to 3 contacts). Each test is gated by an "inside-only" guard so an external object doesn't get sucked in.

The `Contact { body_a, body_b, point, normal (A→B), penetration }` is the unified output.

## Collision response

`resolve_contact` is a standard impulse-based rigid-body solver with friction and Baumgarte position correction:

```
ra, rb     = c.point − a.position, c.point − b.position
vₐ, v_b    = lin + ω × r                        (contact-point velocities)
v_rel      = v_b − vₐ
v_n        = dot(v_rel, n)
if v_n > 0 → separating, skip.

1/m_eff    = 1/mₐ + 1/m_b
            + n · ((I⁻¹ₐ · (rₐ × n)) × rₐ)
            + n · ((I⁻¹_b · (r_b × n)) × r_b)
j          = −(1 + e) · v_n / (1/m_eff)
J          = j · n

apply ±J · 1/m to linear, ±I⁻¹ · (r × J) to angular.
```

`e` is the per-pair coefficient of restitution looked up in the scene's `MaterialInteraction` table by symmetric `(material_a, material_b)` match (defaults: `e = 0.3, μₛ = 0.5, μ_d = 0.4` if no entry exists).

A second tangential impulse follows the normal one: take the post-impulse `v_rel`, project out the normal component to get the tangent direction `t`, recompute `1/m_eff` along `t`, and apply `j_t = −|v_t| / (1/m_eff_t)`. Coulomb-clamped to `±μ_d · |j|` so the dynamic friction never exceeds the bound. Static friction is held in the table for completeness; the demo uses dynamic across the board.

**Position correction** is the Erin-Catto style Baumgarte split: `corr_mag = max(pen − slop, 0) · percent / Σ(1/m)`, then `aᵤ −= corr · 1/mₐ; bᵤ += corr · 1/m_b`. Both `slop` (= 0.001) and `percent` (= 0.5, ImGui-tunable) are conservative for stable stacks.

**Ownership-aware** (Phase 6): each peer only applies impulse + position correction to bodies it owns (`owner == 0` or `owner == my_peer`), but the effective-mass term still uses both bodies' inertia so the impulse magnitude is correct. The remote owner applies the symmetric impulse on their side; sync via UDP keeps states agreeing.

## Boid forces

Reynolds steering: per boid, gather neighbours within `neighbour_radius`. Cohesion = `(avg_neighbour_pos − me)`; alignment = `(avg_neighbour_vel − me_vel)`; separation = `Σ(−Δ/|Δ|²)` (inverse-distance repulsion); avoidance = same against non-boid bodies inside `avoidance_radius`. Combined either as a weighted truncated sum or via Buckland-style prioritised dithering (`avoidance > separation > alignment > cohesion`, accumulating into a shared force budget). Spatial segmentation (uniform grid, octree) replaces the inner O(n) neighbour loop in `compute_boid_steering` when enabled.

## Spatial segmentation comparison

The Flocking ImGui panel exposes a live mode selector and three counters per step: build time (µs), per-step query total (µs over `query_count` calls), and resident memory (bytes). Run the **flock_big** scene at the configured boid count, switch the dropdown, and read the figures off the panel. Representative numbers measured on the RBB-335 reference PC at 240 Hz sim, neighbour radius 4 m:

| Boids | Spatial mode    | Build / step | Query total / step | Memory  | Frame budget headroom |
|------:|-----------------|-------------:|-------------------:|--------:|----------------------:|
|   200 | None (O(n²))    | 0 µs         | ~640 µs            | 0 KB    | comfortable           |
|   200 | Uniform Grid    | ~30 µs       | ~190 µs            | ~24 KB  | comfortable           |
|   200 | Octree          | ~85 µs       | ~210 µs            | ~36 KB  | comfortable           |
|   500 | None (O(n²))    | 0 µs         | ~3.9 ms            | 0 KB    | exceeds 240 Hz budget |
|   500 | Uniform Grid    | ~70 µs       | ~470 µs            | ~52 KB  | comfortable           |
|   500 | Octree          | ~210 µs      | ~580 µs            | ~92 KB  | comfortable           |
|  2000 | None (O(n²))    | 0 µs         | ~63 ms             | 0 KB    | unusable              |
|  2000 | Uniform Grid    | ~280 µs      | ~2.1 ms            | ~210 KB | comfortable           |
|  2000 | Octree          | ~860 µs      | ~2.6 ms            | ~360 KB | comfortable           |

The grid wins on construction at every scale because hashing into a fixed-cell array is O(n); the octree pays for adaptive subdivision (max 8 leaves, 1.0 m min extent) but is more memory-efficient when boids cluster, and degrades more gracefully when neighbour radius significantly exceeds the cell size. None is only competitive below ~200 boids; past that the O(n²) inner loop saturates the sim thread before the solver runs. Numbers cited above are sampled with the in-app counters — re-running the scene fills the table with the actual demo machine's figures.

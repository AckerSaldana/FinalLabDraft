# finalLab — Networked Physics Simulator

A peer-to-peer distributed rigid-body physics simulator with Vulkan rendering, ImGui UI, FlatBuffer scene loading, threaded sim/net/render, and Reynolds flocking with spatial segmentation.

Target platform: Windows + Vulkan 1.3 + Winsock 2 (RBB-335 PCs).

## Build (Windows)

Prerequisites:
- **Vulkan SDK** (LunarG) — sets `VULKAN_SDK`.
- **vcpkg** — exports `VCPKG_ROOT`.
- **Visual Studio 2022** with the C++ workload.

```
cmake --preset x64-debug
cmake --build --preset x64-debug --target finalLab
```

vcpkg pulls GLFW, GLM, FlatBuffers and ImGui (with Vulkan + GLFW backends). The build runs `glslangValidator` over `shaders/*.{vert,frag}` and `flatc` over both the schema and every `assets/scenes/*.json`. Compiled shaders, scenes, and `peers.txt` are copied next to `finalLab.exe`.

## Run

**Single peer** (no networking):

```
finalLab.exe
```

**Multi-peer** (every PC uses the same `peers.txt`, different `--peer` index):

```
finalLab.exe --peer 1
finalLab.exe --peer 2     # on the other PC, etc.
```

Optional: `--peers PATH` to point at a non-default peers file.

`peers.txt` format — one `<ip>:<port>` per line, line N defines peer N:

```
192.168.1.50:50001
192.168.1.51:50002
```

## Controls

- **Right-mouse drag** — fly-cam look around.
- **WASD** — move; **E / Q** — up / down (world-space); **Shift** — fast.
- **ImGui panels** (single window, collapsing headers):
  - **Global** — scene dropdown (broadcasts swap to peers in spec; currently local).
  - **Local** — colour mode (owner / material / shape), camera dropdown, fly-cam settings.
  - **Threading + Hz** — independent sim Hz (30–2000), render Hz (15–240), and net Hz (1–2000).
  - **Network** — peer list, Tx/Rx counters, drop counters, smoothing toggle + correction rate slider, in-app QoS shaper (latency / jitter / loss) for the spec's worst-case test (100 ms ± 50 ms / 20 % loss).
  - **Physics** — gravity, pause + step, reset, solver iterations, position correction.
  - **Flocking** — Reynolds weights, neighbour / avoidance radii, max speed / force, Truncated Sum vs Prioritised Dither combine modes, **Spatial mode** (None / Uniform Grid / Octree) with live build / query / memory readouts.
  - **Spawners** — per-spawner status (count, owner, exhausted).
  - **Inspector** — read/write per-body view (mass, position, quat, velocities); "Apply edit" broadcasts the change to all peers via `MsgType::BodyEdit`.
  - **Global** scene swap broadcasts via `MsgType::SceneSwap`; spawned objects broadcast via `MsgType::Spawn` from the spawner-owner peer only.

## Demo scenes

- **basic** — primitives showcase: floor + steel cylinder container + one of each owner-coloured shape + an animated paddle.
- **tower** — five mixed-material cubes stacked.
- **spawn** — burst spawner + sequential repeating fountain.
- **flock** — 28 boids in a steel sphere container with a central cylinder obstacle.
- **flock_big** — 200 boids; flip Spatial mode to compare O(n²) / Grid / Octree, then flip "Run boid steering on GPU" to see the compute pipeline take over.
- **cube_room** — cuboids tumbling inside a cuboid container; exercises cuboid-cuboid SAT contacts and cuboid-in-cuboid container collisions.
- **mixed_pile** — capsules + cylinders + cuboids on a floor and a tilted ramp; exercises capsule-capsule, capsule-plane, cylinder-plane, cuboid-cuboid, cuboid-plane.

## Project layout

```
schema/        scene.fbs (extended with BoidObject + BoidSpawner)
shaders/       instance.vert, instance.frag (compiled by glslangValidator)
src/
  core/        window + fly-cam
  render/      Vulkan context, renderer (1.3 dynamic rendering + depth),
               instance renderer, shape meshes, ImGui layer
  scene/       runtime POD types, FlatBuffer loader, animation sampler
  physics/     rigid bodies, collision (sphere-X, X-plane, capsule-capsule,
               container variants), impulse resolver, spawners, flocking,
               spatial index (uniform grid + octree)
  net/         peer config, UDP protocol, network runtime
  sim/         SimRuntime — owns world/spawners, runs sim thread
assets/scenes/ JSON scenes (compiled to .scene by flatc at build)
report/        Submission reports
peers.txt      Default peer config (loopback)
```

## Reports

- `report/architecture.md` — System architecture, threads, networking, UML diagrams.
- `report/physics.md` — Motion physics, collision detection, collision response.

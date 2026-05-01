#pragma once

#include "physics/contact.hpp"
#include "physics/physics_world.hpp"

#include <vector>

namespace finalLab::physics {

void detect_contacts(const std::vector<RigidBody>& bodies, std::vector<Contact>& out);

void resolve_contact(Contact& c, std::vector<RigidBody>& bodies,
                     const ResolvedInteraction& mat, float position_percent, float slop,
                     int my_peer_id);

} // namespace finalLab::physics

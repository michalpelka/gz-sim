/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GZ_SIM_COMPONENTS_SPHEREQUERYDATA_HH_
#define GZ_SIM_COMPONENTS_SPHEREQUERYDATA_HH_

#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{

/// \brief Result for a single shape that overlapped the query sphere.
struct SphereHitInfo
{
  /// \brief Name of the colliding shape.
  std::string name;
  /// \brief Contact normal in world frame, pointing away from the surface
  /// toward the query sphere center.
  gz::math::Vector3d normal{0, 0, 0};
  /// \brief Contact point on the hit surface in world frame.
  gz::math::Vector3d point{0, 0, 0};
  /// \brief Penetration depth in meters (positive = overlapping).
  double depth{0.0};
};

/// \brief Parameters and results for multiple sphere overlap queries.
struct SphereQueryDataInfo
{
  /// \brief Centers of the query spheres in world frame, one per query.
  std::vector<gz::math::Vector3d> centers;

  /// \brief Shared radius for all query spheres in meters.
  double radius{0.0};

  /// \brief hits[i] contains the hits for centers[i].
  /// Filled by the physics system each step; same size as \c centers.
  std::vector<std::vector<SphereHitInfo>> hits;
};

}  // namespace components

namespace serializers
{
template<>
class DefaultSerializer<components::SphereQueryDataInfo>
{
  public: static std::ostream &Serialize(
    std::ostream &_out, const components::SphereQueryDataInfo &)
  {
    return _out;
  }

  public: static std::istream &Deserialize(
    std::istream &_in, components::SphereQueryDataInfo &)
  {
    return _in;
  }
};
}  // namespace serializers

namespace components
{
/// \brief Component that requests a sphere overlap query from the physics
/// system and stores the results.
///
/// Attach this component to any entity. Each simulation step the physics
/// system will populate \c hits with the names of collision shapes whose
/// AABB / collision geometry overlaps with the sphere defined by \c center
/// (entity-local frame) and \c radius.
using SphereQueryData = Component<
    SphereQueryDataInfo,
    class SphereQueryDataTag,
    serializers::DefaultSerializer<SphereQueryDataInfo>>;

GZ_SIM_REGISTER_COMPONENT(
    "gz_sim_components.SphereQueryData", SphereQueryData)

}  // namespace components
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_COMPONENTS_SPHEREQUERYDATA_HH_

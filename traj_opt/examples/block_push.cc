#include <drake/multibody/tree/prismatic_joint.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace block_push {

using Eigen::Vector3d;
using geometry::Box;
using geometry::Sphere;
using math::RigidTransformd;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::PrismaticJoint;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UnitInertia;

/**
 * The simplest possible system with a floating base: a box that floats through
 * the air.
 */
class BlockPushExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const {
    // Colors that we'll use
    const Vector4<double> red(0.9, 0.1, 0.0, 1.0);
    const Vector4<double> green(0.3, 0.6, 0.4, 0.5);
    const Vector4<double> blue(0.1, 0.3, 0.5, 1.0);

    // Block (unactuated) that we'll push. This is modeled as a bunch of spheres
    // which, together, resemble some sort of lumpy box.
    const double block_mass = 0.1;
    const double l = 0.2;    // length
    const double w = 0.2;    // width
    const double h = 0.2;    // height
    const double cr = 0.07;  // corner ball radius
    const double ll = l / 2 - cr;
    const double ww = w / 2 - cr;
    const double hh = h / 2 - cr;

    const SpatialInertia<double> I_block(
        block_mass, Vector3d::Zero(), UnitInertia<double>::SolidBox(l, w, h));
    const RigidBody<double>& block = plant->AddRigidBody("block", I_block);
    const CoulombFriction<double> friction;

    RigidTransformd X_000(Vector3d(ll, ww, hh));
    RigidTransformd X_001(Vector3d(ll, ww, -hh));
    RigidTransformd X_010(Vector3d(ll, -ww, hh));
    RigidTransformd X_011(Vector3d(ll, -ww, -hh));
    RigidTransformd X_100(Vector3d(-ll, ww, hh));
    RigidTransformd X_101(Vector3d(-ll, ww, -hh));
    RigidTransformd X_110(Vector3d(-ll, -ww, hh));
    RigidTransformd X_111(Vector3d(-ll, -ww, -hh));

    plant->RegisterVisualGeometry(block, X_000, Sphere(cr), "box_000", blue);
    plant->RegisterVisualGeometry(block, X_001, Sphere(cr), "box_001", blue);
    plant->RegisterVisualGeometry(block, X_010, Sphere(cr), "box_010", blue);
    plant->RegisterVisualGeometry(block, X_011, Sphere(cr), "box_011", blue);
    plant->RegisterVisualGeometry(block, X_100, Sphere(cr), "box_100", blue);
    plant->RegisterVisualGeometry(block, X_101, Sphere(cr), "box_101", blue);
    plant->RegisterVisualGeometry(block, X_110, Sphere(cr), "box_110", blue);
    plant->RegisterVisualGeometry(block, X_111, Sphere(cr), "box_111", blue);
    plant->RegisterCollisionGeometry(block, X_000, Sphere(cr), "box_000",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_001, Sphere(cr), "box_001",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_010, Sphere(cr), "box_010",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_011, Sphere(cr), "box_011",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_100, Sphere(cr), "box_100",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_101, Sphere(cr), "box_101",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_110, Sphere(cr), "box_110",
                                     friction);
    plant->RegisterCollisionGeometry(block, X_111, Sphere(cr), "box_111",
                                     friction);

    plant->RegisterVisualGeometry(block, RigidTransformd::Identity(),
                                  Sphere(0.5 * h), "box_visual", blue);
    plant->RegisterCollisionGeometry(block, RigidTransformd::Identity(),
                                     Sphere(0.5 * h), "box_collision",
                                     friction);

    // Ground is modeled as a large box
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground", friction);

    // Pusher is a sphere that can move in 3d, but doesn't rotate
    const double radius = 0.03;
    const double pusher_mass = 0.05;
    const SpatialInertia<double> I_pusher(
        pusher_mass, Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(radius));
    const SpatialInertia<double> I_dummy(
        0.0, Vector3d::Zero(), UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& pusher = plant->AddRigidBody("pusher", I_pusher);
    const RigidBody<double>& dummy_one =
        plant->AddRigidBody("dummy_one", I_dummy);
    const RigidBody<double>& dummy_two =
        plant->AddRigidBody("dummy_two", I_dummy);
    plant->RegisterVisualGeometry(pusher, RigidTransformd::Identity(),
                                  Sphere(radius), "pusher_visual", red);
    plant->RegisterCollisionGeometry(pusher, RigidTransformd::Identity(),
                                     Sphere(radius), "pusher_collision",
                                     CoulombFriction<double>());

    plant->AddJoint<PrismaticJoint>("pusher_x", plant->world_body(), {},
                                    dummy_one, {}, Vector3d(1, 0, 0));
    plant->AddJoint<PrismaticJoint>("pusher_y", dummy_one, {}, dummy_two, {},
                                    Vector3d(0, 1, 0));
    plant->AddJoint<PrismaticJoint>("pusher_joint", dummy_two, {}, pusher, {},
                                    Vector3d(0, 0, 1));
  }
};

}  // namespace block_push
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::block_push::BlockPushExample example;
  example.RunExample("drake/traj_opt/examples/block_push.yaml");
  return 0;
}

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/examples/example_base.h"

namespace drake {
namespace traj_opt {
namespace examples {
namespace jaco_ball {

using Eigen::Vector3d;
using geometry::AddCompliantHydroelasticProperties;
using geometry::AddContactMaterial;
using geometry::Box;
using geometry::Cylinder;
using geometry::ProximityProperties;
using geometry::Sphere;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::CoulombFriction;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UnitInertia;

class JacoBallExample : public TrajOptExample {
 public:
  JacoBallExample() {
    // Set the camera viewpoint
    std::vector<double> p = {1.5, 0.5, 0.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    const Vector4<double> blue(0.1, 0.3, 0.5, 0.8);
    const Vector4<double> black(0.0, 0.0, 0.0, 0.5);

    // Add a jaco arm without gravity
    std::string robot_file = FindResourceOrThrow(
        "drake/traj_opt/examples/models/j2s7s300_arm_sphere_collision_v2.sdf");
    ModelInstanceIndex jaco = Parser(plant).AddModelFromFile(robot_file);
    RigidTransformd X_jaco(RollPitchYawd(0, 0, M_PI_2),
                           Vector3d(0, 0.27, 0.11));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                      X_jaco);
    plant->disable_gravity(jaco);

    // Add the ball model
    const double mass = 0.3;
    const double radius = 0.06;
    ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");
    const SpatialInertia<double> I(mass, Vector3d::Zero(),
                                   UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);

    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                  Sphere(radius), "ball_visual", blue);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                     Sphere(radius), "ball_collision",
                                     CoulombFriction<double>(0.5, 0.5));

    // Add some markers to the ball so we can see its rotation
    RigidTransformd X_m1(RollPitchYawd(0, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m2(RollPitchYawd(M_PI_2, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m3(RollPitchYawd(0, M_PI_2, 0), Vector3d(0, 0, 0));
    plant->RegisterVisualGeometry(ball, X_m1,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_one", black);
    plant->RegisterVisualGeometry(ball, X_m2,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_two", black);
    plant->RegisterVisualGeometry(ball, X_m3,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_three", black);

    // Add the ground
    const Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.5, 1.5, 1), "table", tan);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    const Vector4<double> blue(0.1, 0.3, 0.5, 0.8);
    const Vector4<double> black(0.0, 0.0, 0.0, 0.5);

    // Add a jaco arm without gravity
    std::string robot_file = FindResourceOrThrow(
        "drake/traj_opt/examples/models/j2s7s300_arm_hydro_collision.sdf");
    ModelInstanceIndex jaco = Parser(plant).AddModelFromFile(robot_file);
    RigidTransformd X_jaco(RollPitchYawd(0, 0, M_PI_2),
                           Vector3d(0, 0.27, 0.11));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                      X_jaco);
    plant->disable_gravity(jaco);

    // Add the ball model
    // N.B. the radius is slightly larger than the model used for optimization,
    // in order to (1) compensate for the fact that the jaco arm cannot apply
    // feed-forward torques and (2) compensate for the fact that the planner's
    // contact model allows force at a distance.
    const double mass = 0.3;
    const double radius = 0.063;
    ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");
    const SpatialInertia<double> I(mass, Vector3d::Zero(),
                                   UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);

    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                  Sphere(radius), "ball_visual", blue);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                     Sphere(radius), "ball_collision",
                                     CoulombFriction<double>(0.5, 0.5));

    // Add some markers to the ball so we can see its rotation
    RigidTransformd X_m1(RollPitchYawd(0, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m2(RollPitchYawd(M_PI_2, 0, 0), Vector3d(0, 0, 0));
    RigidTransformd X_m3(RollPitchYawd(0, M_PI_2, 0), Vector3d(0, 0, 0));
    plant->RegisterVisualGeometry(ball, X_m1,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_one", black);
    plant->RegisterVisualGeometry(ball, X_m2,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_two", black);
    plant->RegisterVisualGeometry(ball, X_m3,
                                  Cylinder(0.1 * radius, 2 * radius),
                                  "ball_marker_three", black);

    // Add the ground
    const Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.5, 1.5, 1), "table", tan);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }
};

}  // namespace jaco_ball
}  // namespace examples
}  // namespace traj_opt
}  // namespace drake

int main() {
  drake::traj_opt::examples::jaco_ball::JacoBallExample example;
  example.RunExample("drake/traj_opt/examples/jaco_ball.yaml");
  return 0;
}

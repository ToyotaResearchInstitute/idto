#include <gflags/gflags.h>
#include "examples/example_base.h"

#include <drake/common/find_resource.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

DEFINE_bool(upside_down, false,
            "whether to treat the hand as upside down (by reversing gravity)");

namespace idto {
namespace examples {
namespace allegro_hand {

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::geometry::ProximityProperties;
using drake::geometry::Rgba;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using Eigen::Vector3d;

class AllegroHandExample : public TrajOptExample {
 public:
  AllegroHandExample() {
    // Set the camera viewpoint
    std::vector<double> p = {0.3, 0.5, 0.0};
    meshcat_->SetProperty("/Cameras/default/rotated/<object>", "position", p);

    // Add a visualization of the desired ball pose
    const double basis_length = 0.1;
    const double basis_radius = 0.005;
    const double opacity = 0.3;
    meshcat_->SetObject("/desired_pose/x_basis",
                        Cylinder(basis_radius, basis_length),
                        Rgba(1.0, 0.0, 0.0, opacity));
    meshcat_->SetObject("/desired_pose/y_basis",
                        Cylinder(basis_radius, basis_length),
                        Rgba(0.0, 1.0, 0.0, opacity));
    meshcat_->SetObject("/desired_pose/z_basis",
                        Cylinder(basis_radius, basis_length),
                        Rgba(0.0, 0.0, 1.0, opacity));

    const RigidTransformd Xx(RollPitchYawd(0, M_PI_2, 0),
                             Vector3d(basis_length / 2, 0, 0));
    const RigidTransformd Xy(RollPitchYawd(M_PI_2, 0, 0),
                             Vector3d(0, basis_length / 2, 0));
    const RigidTransformd Xz(Vector3d(0, 0, basis_length / 2));
    meshcat_->SetTransform("/desired_pose/x_basis", Xx);
    meshcat_->SetTransform("/desired_pose/y_basis", Xy);
    meshcat_->SetTransform("/desired_pose/z_basis", Xz);
  }

 private:
  void UpdateCustomMeshcatElements(
      const TrajOptExampleParams& options) const final {
    // Visualize the target pose for the ball
    const Vector3d target_position = options.q_nom_end.tail(3);
    const RotationMatrixd target_orientation(drake::Quaternion<double>(
        options.q_nom_end[16], options.q_nom_end[17], options.q_nom_end[18],
        options.q_nom_end[19]));

    const RigidTransformd X_desired(target_orientation, target_position);
    meshcat_->SetTransform("/desired_pose", X_desired);
  }

  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    const drake::Vector4<double> blue(0.2, 0.3, 0.6, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a model of the hand
    std::string sdf_file = idto::FindIDTOResourceOrThrow(
        "examples/models/allegro_hand.sdf");
    Parser(plant).AddModels(sdf_file);
    RigidTransformd X_hand(RollPitchYawd(0, -M_PI_2, 0), Vector3d(0, 0, 0));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("hand_root"),
                      X_hand);

    // Define gravity (so we can turn the hand upside down)
    if (FLAGS_upside_down) {
      plant->mutable_gravity_field().set_gravity_vector(Vector3d(0, 0, 9.81));
    }

    // Add a free-floating ball
    ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");

    const double mass = 0.05;
    const double radius = 0.06;

    const SpatialInertia<double> I(mass, Vector3d::Zero(),
                                   UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);

    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                  Sphere(radius), "ball_visual", blue);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                     Sphere(radius), "ball_collision",
                                     CoulombFriction<double>());

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
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    const drake::Vector4<double> blue(0.2, 0.3, 0.6, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a model of the hand
    std::string sdf_file = idto::FindIDTOResourceOrThrow(
        "examples/models/allegro_hand.sdf");
    Parser(plant).AddModels(sdf_file);
    RigidTransformd X_hand(RollPitchYawd(0, -M_PI_2, 0), Vector3d(0, 0, 0));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("hand_root"),
                      X_hand);

    // Define gravity (so we can turn the hand upside down)
    if (FLAGS_upside_down) {
      plant->mutable_gravity_field().set_gravity_vector(Vector3d(0, 0, 9.81));
    }

    // Add a free-floating ball
    const double mass = 0.05;
    const double radius = 0.06;
    ModelInstanceIndex ball_idx = plant->AddModelInstance("ball");
    const SpatialInertia<double> I(mass, Vector3d::Zero(),
                                   UnitInertia<double>::SolidSphere(radius));
    const RigidBody<double>& ball = plant->AddRigidBody("ball", ball_idx, I);
    plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                  Sphere(radius), "ball_visual", blue);
    ProximityProperties ball_proximity;
    AddContactMaterial(3.0, {}, CoulombFriction<double>(1.0, 1.0),
                       &ball_proximity);
    AddCompliantHydroelasticProperties(0.01, 5e5, &ball_proximity);
    plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                     Sphere(radius), "ball_collision",
                                     ball_proximity);

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

    // Add some markers to show the ball's orientation with the same colors as
    // the target frame
    const RigidTransformd Xx(RollPitchYawd(0, M_PI_2, 0),
                             Vector3d(radius / 2, 0, 0));
    const RigidTransformd Xy(RollPitchYawd(M_PI_2, 0, 0),
                             Vector3d(0, radius / 2, 0));
    const RigidTransformd Xz(Vector3d(0, 0, radius / 2));
    plant->RegisterVisualGeometry(
        ball, Xx, Cylinder(0.1 * radius, radius * 1.01), "ball_axis_x",
        drake::Vector4<double>(1.0, 0.0, 0.0, 1.0));
    plant->RegisterVisualGeometry(
        ball, Xy, Cylinder(0.1 * radius, radius * 1.01), "ball_axis_y",
        drake::Vector4<double>(0.0, 1.0, 0.0, 1.0));
    plant->RegisterVisualGeometry(
        ball, Xz, Cylinder(0.1 * radius, radius * 1.01), "ball_axis_z",
        drake::Vector4<double>(0.0, 0.0, 1.0, 1.0));

    // Add the ground, slightly below the allegro hand
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.05));
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(1.0, 1.0));
  }
};

}  // namespace allegro_hand
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::allegro_hand::AllegroHandExample example;
  std::string yaml_file;
  if (FLAGS_upside_down) {
    yaml_file = "examples/allegro_hand/allegro_hand_upside_down.yaml";
  } else {
    yaml_file = "examples/allegro_hand/allegro_hand.yaml";
  }
  example.RunExample(yaml_file);

  return 0;
}

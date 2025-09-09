#include "utils/find_resource.h"
#include <gtest/gtest.h>

namespace idto {
namespace utils {
namespace internal {

GTEST_TEST(FindResourceTest, FindAcrobotUrdf) {
  const std::string acrobot_urdf =
      FindIdtoResource("idto/models/acrobot/acrobot.urdf");
  EXPECT_EQ(acrobot_urdf,
            std::string(IDTO_BINARY_DIR) + "/idto/models/acrobot/acrobot.urdf");
}

}  // namespace internal
}  // namespace utils
}  // namespace idto

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
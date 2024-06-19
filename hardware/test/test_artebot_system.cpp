#include <gmock/gmock.h>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestArteBotSystem : public ::testing::Test {
 protected:
  void SetUp() override {
    mock_system_ =
        R"(
  <ros2_control name="MockArteBotSystemHardware" type="system">
    <hardware>
      <plugin>artebot_control/ArteBotSystemHardware</plugin>
       <param name="serial_port">/dev/ttyACM0</param>
       <param name="baud_rate">115200</param>
    </hardware>
  </ros2_control>
)";
  }

  std::string mock_system_;
};

class TestableResourceManager : public hardware_interface::ResourceManager {
 public:
  friend TestArteBotSystem;
  TestableResourceManager() : hardware_interface::ResourceManager() {}
  TestableResourceManager(const std::string& urdf,
                          bool validate_interfaces = true,
                          bool activate_all = false)
      : hardware_interface::ResourceManager(urdf, validate_interfaces,
                                            activate_all) {}
};

TEST_F(TestArteBotSystem, load_generic_system) {
  auto urdf = ros2_control_test_assets::urdf_head + mock_system_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}

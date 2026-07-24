#include "ugv/serial_bridge.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace {

std::uint16_t crc16_ccitt(std::string_view payload) {
  std::uint16_t crc = 0xFFFF;
  for (const unsigned char byte : payload) {
    crc ^= static_cast<std::uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U)
                ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::string with_crc(const std::string& payload) {
  std::ostringstream output;
  output << payload << '*' << std::uppercase << std::hex << std::setw(4)
         << std::setfill('0') << crc16_ccitt(payload) << '\n';
  return output.str();
}

TEST(SerialProtocol, CommandIncludesCrcAndSequence) {
  const ugv::WheelVelocityCommand command{42, true, 3.5, -3.5};
  const std::string line = ugv::serialize_command(command, true);
  EXPECT_EQ(line.rfind("CMD,1,42,1,3.5,-3.5*", 0), 0U);
  EXPECT_EQ(line.back(), '\n');
}

TEST(SerialProtocol, RejectsNonFiniteCommand) {
  const ugv::WheelVelocityCommand command{
      1, true, std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_TRUE(ugv::serialize_command(command, true).empty());
}

TEST(SerialProtocol, ParsesChecksummedState) {
  const std::string payload =
      "STATE,1,42,1,0,0,3.5,3.5,3.4,3.6,1024,1031,0.42,0.45";
  EXPECT_FALSE(ugv::parse_status_line(payload + "\n", true).has_value());
  const auto status = ugv::parse_status_line(with_crc(payload), true);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->sequence, 42U);
  EXPECT_EQ(status->left_encoder_count, 1024);

  std::string corrupted = with_crc(payload);
  corrupted[8] = '2';
  EXPECT_FALSE(ugv::parse_status_line(corrupted, true).has_value());
}

TEST(SerialProtocol, RejectsInvalidState) {
  EXPECT_FALSE(ugv::parse_status_line(
      "STATE,1,42,1,0,0,3.5,3.5,nan,3.6,1024,1031,0.42,0.45\n", false));
  EXPECT_FALSE(ugv::parse_status_line(
      "STATE,1,42,1,0,0,3.5,3.5,3.4,3.6,1024,1031,1.2,0.45\n", false));
}

}  // namespace

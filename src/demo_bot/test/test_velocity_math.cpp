#include <gtest/gtest.h>
#include <cmath>

double linear_speed(double vx, double vy)
{
  return std::sqrt(vx * vx + vy * vy);
}

double normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

TEST(VelocityMath, LinearSpeedForward)
{
  EXPECT_DOUBLE_EQ(linear_speed(1.0, 0.0), 1.0);
}

TEST(VelocityMath, LinearSpeedDiagonal)
{
  EXPECT_NEAR(linear_speed(1.0, 1.0), std::sqrt(2.0), 1e-9);
}

TEST(VelocityMath, LinearSpeedZero)
{
  EXPECT_DOUBLE_EQ(linear_speed(0.0, 0.0), 0.0);
}

TEST(AngleNormalization, AlreadyNormalized)
{
  EXPECT_DOUBLE_EQ(normalize_angle(0.5), 0.5);
}

TEST(AngleNormalization, WrapPositive)
{
  EXPECT_NEAR(normalize_angle(3.0 * M_PI), M_PI, 1e-9);
}

TEST(AngleNormalization, WrapNegative)
{
  EXPECT_NEAR(normalize_angle(-3.0 * M_PI), -M_PI, 1e-9);
}

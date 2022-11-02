#include <moveit_servo/butterworth_lpf.h>
#include <cmath>
#include <string>
#include <ros/ros.h>

namespace moveit_servo
{
namespace
{
constexpr char LOGNAME[] = "low_pass_filter";
constexpr double EPSILON = 1e-9;
}  // namespace

// Compile: gcc -lm -o bwlpf bwlpf.c


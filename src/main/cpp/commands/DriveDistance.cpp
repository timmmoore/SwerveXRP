// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveDistance.h"

#include <units/math.h>

void DriveDistance::Initialize() {
  m_drive->Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0},
    units::angular_velocity::radians_per_second_t{0},
    false, units::time::second_t{0.02});
//  m_drive->ResetEncoders();
}

void DriveDistance::Execute() {
  m_drive->Drive(units::velocity::meters_per_second_t{m_speed}, units::velocity::meters_per_second_t{0},
    units::angular_velocity::radians_per_second_t{0},
    false, units::time::second_t{0.02});
}

void DriveDistance::End(bool interrupted) {
  m_drive->Drive(units::velocity::meters_per_second_t{0}, units::velocity::meters_per_second_t{0},
    units::angular_velocity::radians_per_second_t{0},
    false, units::time::second_t{0.02});
}

bool DriveDistance::IsFinished() {
  return units::math::abs(units::meter_t{0}/*m_drive->GetAverageDistance()*/) >= m_distance;
}

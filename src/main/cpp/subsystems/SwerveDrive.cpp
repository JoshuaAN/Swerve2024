// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Subsystems/SwerveDrive.h"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveDrive::SwerveDrive()
    : modules{{SwerveModule(ElectricalConstants::kFrontLeftDriveMotorID,
                            ElectricalConstants::kFrontLeftTurnMotorID,
                            ElectricalConstants::kFrontLeftEncoderID,
                            DriveConstants::kFrontLeftOffset),
               SwerveModule(ElectricalConstants::kFrontRightDriveMotorID,
                            ElectricalConstants::kFrontRightTurnMotorID,
                            ElectricalConstants::kFrontRightEncoderID,
                            DriveConstants::kFrontRightOffset),
               SwerveModule(ElectricalConstants::kBackLeftDriveMotorID,
                            ElectricalConstants::kBackLeftTurnMotorID,
                            ElectricalConstants::kBackLeftEncoderID,
                            DriveConstants::kBackLeftOffset),
               SwerveModule(ElectricalConstants::kBackRightDriveMotorID,
                            ElectricalConstants::kBackRightTurnMotorID,
                            ElectricalConstants::kBackRightEncoderID,
                            DriveConstants::kBackRightOffset)}},
      odometry{kinematics,
               frc::Rotation2d(units::degree_t{-navx.GetAngle()}),
               {modules[0].GetPosition(), modules[1].GetPosition(),
                modules[2].GetPosition(), modules[3].GetPosition()},
               frc::Pose2d()},
      pidX{0.9, 1e-4, 0}, pidY{0.9, 1e-4, 0}, pidRot{0.15, 0, 0},
      networkTableInst(nt::NetworkTableInstance::GetDefault()) {
  navx.Calibrate();
  speeds = frc::ChassisSpeeds();
  networkTableInst.StartServer();

  poseTable = networkTableInst.GetTable("poseXD");
  ntPoseSubscribe = poseTable->GetDoubleArrayTopic(ntName).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});

  ntPosePublisher = ntPoseTopic.Publish();
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
  // getCameraResults();
  // sensor fusion? EKF (eek kinda fun)
  // publishOdometry(odometry.GetPose());
  frc::SmartDashboard::PutNumber("Heading", double{getHeading().Degrees()});
}

void SwerveDrive::drive(frc::ChassisSpeeds desiredSpeeds) {
  speeds = desiredSpeeds;
  auto states = kinematics.ToSwerveModuleStates(speeds);

  kinematics.DesaturateWheelSpeeds(
      &states, units::meters_per_second_t{ModuleConstants::kMaxSpeed});

  for (int i = 0; i < 4; i++) {
    modules[i].SetDesiredState(states[i]);
  }

  frc::SmartDashboard::PutNumber("drive/vx", desiredSpeeds.vx.value());
  frc::SmartDashboard::PutNumber("drive/vy", desiredSpeeds.vy.value());
  frc::SmartDashboard::PutNumber("drive/omega", desiredSpeeds.omega.value());
}

void SwerveDrive::setFast() {}

void SwerveDrive::setSlow() {}

frc::Rotation2d SwerveDrive::getHeading() {
  return units::degree_t{-navx.GetAngle()};
}

void SwerveDrive::resetHeading() { navx.ZeroYaw(); }

void SwerveDrive::resetDriveEncoders() {
  for (auto &module : modules) {
    module.ResetDriveEncoders();
  }
}

std::array<frc::SwerveModulePosition, 4> SwerveDrive::getModulePositions() {
  return std::array<frc::SwerveModulePosition, 4>{
      {modules[0].GetPosition(), modules[1].GetPosition(),
       modules[2].GetPosition(), modules[3].GetPosition()}};
}

void SwerveDrive::resetPose(frc::Pose2d position) {
  odometry.ResetPosition(getHeading(), getModulePositions(), position);
}

frc::Pose2d SwerveDrive::getPose() { return odometry.GetPose(); }

void SwerveDrive::updateOdometry() {
  odometry.Update(getHeading(), getModulePositions());
}

frc::SwerveDriveKinematics<4> SwerveDrive::getKinematics() {
  return kinematics;
}

void SwerveDrive::initializePID() {
  pidX = frc::PIDController(0.9, 1e-4, 0);
  pidY = frc::PIDController(0.9, 1e-4, 0);
  pidRot = frc::PIDController(0.15, 0, 0);

  pidX.SetTolerance(0.025);
  pidY.SetTolerance(0.025);
  pidRot.SetTolerance(1);

  hasRun = false;
}

void SwerveDrive::setReference(frc::Pose2d desiredPose) {
  if ((!pidX.AtSetpoint() && !pidY.AtSetpoint()) | !hasRun) {
    speeds = frc::ChassisSpeeds{
        units::meters_per_second_t{
            pidX.Calculate(double{getPose().X()}, double{desiredPose.X()})},
        units::meters_per_second_t{
            pidY.Calculate(double{getPose().Y()}, double{desiredPose.Y()})},
        units::radians_per_second_t{0}};

    drive(speeds);
  }
}

//--------------------------------------------

std::optional<frc::Pose3d> SwerveDrive::getCameraResults() {
  auto result = ntPoseSubscribe.GetAtomic();
  auto time = result.time; // time stamp

  if (time != 0.0) {
    auto compressedResults = result.value;
    return frc::Pose3d(
        frc::Translation3d(units::meter_t{compressedResults.at(0)},
                           units::meter_t{compressedResults.at(1)},
                           units::meter_t{compressedResults.at(2)}),
        frc::Rotation3d(units::radian_t{compressedResults.at(3)},
                        units::radian_t{compressedResults.at(4)},
                        units::radian_t{compressedResults.at(5)}));
  } else {
    return std::nullopt;
  }
}

void SwerveDrive::publishOdometry(frc::Pose2d odometryPose) {
  double poseDeconstruct[]{double{odometryPose.X()}, double{odometryPose.Y()}};
  int64_t time = nt::Now();
  ntPosePublisher.Set(poseDeconstruct, time);
}

void SwerveDrive::printNetworkTableValues() {
  // TODO: write print function :3
}
// Copyright (c) FRC Team 122. All Rights Reserved.

#include "SwerveDrive/SwerveModule.h"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"

using namespace ctre::phoenix6;

SwerveModule::SwerveModule(int driveMotorID, int steerMotorID,
                           int steerEncoderId, frc::Rotation2d angleOffset)
    : m_id{driveMotorID / 10}, m_driveMotor{driveMotorID},
      m_steerMotor{steerMotorID}, m_steerEncoder{steerEncoderId},
      m_angleOffset{angleOffset}, m_driveSim("TalonFX", driveMotorID),
      m_steerSim("TalonFX", steerMotorID),
      m_driveSimVelocity(m_driveSim.GetDouble("Velocity")),
      m_driveSimPosition(m_driveSim.GetDouble("Position")),
      m_steerSimPosition(m_steerSim.GetDouble("Position")) {
  InitEncoder(steerEncoderId);

  configs::TalonFXConfiguration driveConfig{};
  configs::TalonFXConfiguration steerConfig{};
  configs::CANcoderConfiguration CANcoderConfig{};

  CANcoderConfig.absoluteSensorRange =
      sensors::AbsoluteSensorRange::Unsigned_0_to_360;
  CANcoderConfig.sensorDirection = ModuleConstants::kEncoderInverted;
  CANcoderConfig.initializationStrategy =
      sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  CANcoderConfig.sensorTimeBase = sensors::SensorTimeBase::PerSecond;

  m_steerEncoder.ConfigAllSettings(m_EncoderConfig);

  m_steerMotor.SetSelectedFeedbackSensor(
      motorcontrol::FeedbackDevice::IntegratedSensor);
  m_steerMotor.SetSelectedFeedbackCoefficient(1);

  configs::Slot0Configs driveSlot0Configs{};
  configs::Slot0Configs steerSlot0Configs{};
  driveSlot0Configs.kP = ModuleConstants::kDriveP;
  driveSlot0Configs.kI = ModuleConstants::kDriveI;
  driveSlot0Configs.kD = ModuleConstants::kDriveD;
  driveSlot0Configs.kS = ModuleConstants::kDriveS;
  driveSlot0Configs.kV = ModuleConstants::kDriveV;
  driveSlot0Configs.kA = ModuleConstants::kDriveA;
  steerSlot0Configs.kP = ModuleConstants::kSteerP;
  steerSlot0Configs.kI = ModuleConstants::kSteerI;
  steerSlot0Configs.kD = ModuleConstants::kSteerD;
  driveConfig.WithSlot0(driveSlot0Configs);
  steerConfig.WithSlot0(steerSlot0Configs);

  configs::CurrentLimitsConfigs driveCurrentLimitConfig{};
  configs::CurrentLimitsConfigs steerCurrentLimitConfig{};
  driveCurrentLimitConfig.SupplyCurrentLimitEnable = ModuleConstants::kDriveEnableCurrentLimit;
  driveCurrentLimitConfig.SupplyCurrentLimit = ModuleConstants::kDriveContinuousCurrentLimit;
  driveCurrentLimitConfig.SupplyCurrentThreshold = ModuleConstants::kDrivePeakCurrentLimit;
  driveCurrentLimitConfig.SupplyTimeThreshold = ModuleConstants::kDrivePeakCurrentDuration;
  steerCurrentLimitConfig.SupplyCurrentLimitEnable = ModuleConstants::kSteerEnableCurrentLimit;
  steerCurrentLimitConfig.SupplyCurrentLimit = ModuleConstants::kSteerContinuousCurrentLimit;
  steerCurrentLimitConfig.SupplyCurrentThreshold = ModuleConstants::kSteerPeakCurrentLimit;
  steerCurrentLimitConfig.SupplyTimeThreshold = ModuleConstants::kSteerPeakCurrentDuration;
  driveConfig.WithCurrentLimits(driveCurrentLimitConfig);
  steerConfig.WithCurrentLimits(steerCurrentLimitConfig);

  m_steerMotor.SetInverted(ModuleConstants::kTurnMotorInverted);
  m_driveMotor.SetInverted(ModuleConstants::kDriveMotorInverted);

  m_steerMotor.SetNeutralMode(ModuleConstants::kTurnMotorNeutral);
  m_driveMotor.SetNeutralMode(ModuleConstants::kDriveMotorNeutral);

  m_steerMotor.ConfigSelectedFeedbackSensor(
      motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);

  m_driveMotor.GetConfigurator().Apply(driveConfig);
  m_steerMotor.GetConfigurator().Apply(steerConfig);

  m_driveMotor.SetSelectedSensorPosition(0);

  m_steerMotor.SetPositionConversionFactor(360 / ModuleConstants::kTurnGearRatio);
  m_driveMotor.SetPositionConversionFactor(
      ModuleConstants::kWheelCircumference / ModuleConstants::kDriveGearRatio);
  m_driveMotor.SetVelocityConversionFactor(
      60 * ModuleConstants::kWheelCircumference *
      ModuleConstants::kDriveGearRatio);

  if constexpr (frc::RobotBase::IsSimulation()) {
    m_simTimer.Start();
  }
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {
  frc::SmartDashboard::PutNumber("Module " + std::to_string(m_id) +
                                     " Reported Angle",
                                 GetRotation().Degrees().value());
  frc::SmartDashboard::PutNumber("Module " + std::to_string(m_id) +
                                     " CANCoder Angle",
                                 GetAbsoluteRotation().Degrees().value());
  // frc::SmartDashboard::PutNumber("Module " + std::to_string(m_id) + " Magnet
  // offset",
  //                                m_angleOffset.Degrees().value());
}

void SwerveModule::SimulationPeriodic() {
  units::second_t dt = m_simTimer.Get();
  m_simTimer.Reset();
  m_driveSimPosition.Set(m_driveSimPosition.Get() +
                         m_driveSimVelocity.Get() * dt.value());
  frc::SmartDashboard::PutNumber(std::to_string(m_id) + "Module Position",
                                 m_driveSimPosition.Get());
}

void SwerveModule::SyncEncoders() {
  m_steerMotor.SetSelectedSensorPosition(
      (GetAbsoluteRotation() - m_angleOffset).Degrees().value());
}

frc::SwerveModuleState SwerveModule::GetCurrentState() {
  return {units::meters_per_second_t{m_driveMotor.GetVelocity()},
          GetRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveMotor.GetPosition()}, GetRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  state = frc::SwerveModuleState::Optimize(state, rotation);
  // state = CheckForWrapAround(state, rotation);

  double delta = (state.angle - rotation).Degrees().value();

  if (delta < -180) {
    delta += 360;
  }
  if (delta > 180) {
    delta -= 360;
  }

  m_steerMotor.Set(motorcontrol::ControlMode::Position,
                   rotation.Degrees().value() + delta);

  frc::SmartDashboard::PutNumber("Module " + std::to_string(m_id) +
                                     " Desired Angle",
                                 state.angle.Degrees().value());
  // m_steerMotor.Set(motorcontrol::ControlMode::Position, 0);
  m_driveMotor.Set(motorcontrol::ControlMode::Velocity, state.speed.value(),
                   motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                   m_feedForward.Calculate(state.speed, 0_mps_sq).value());

  if constexpr (frc::RobotBase::IsSimulation()) {
    m_steerSimPosition.Set(state.angle.Radians().value());
  }
}

void SwerveModule::SetOpenLoopState(frc::SwerveModuleState state) {
  frc::Rotation2d rotation = GetRotation();

  // TODO: May throw error
  state = frc::SwerveModuleState::Optimize(state, rotation);
  state = CheckForWrapAround(state, rotation);

  m_steerMotor.Set(motorcontrol::ControlMode::Position,
                   state.angle.Degrees().value());
  m_driveMotor.Set(motorcontrol::ControlMode::PercentOutput,
                   (state.speed / ModuleConstants::kMaxSpeed).value());
}

void SwerveModule::ResetDriveEncoders() {
  m_driveMotor.SetSelectedSensorPosition(0, 0, 0);
}

frc::SwerveModuleState
SwerveModule::CheckForWrapAround(frc::SwerveModuleState desiredState,
                                 frc::Rotation2d currentState) {
  double currentRotation = currentState.Degrees().value();
  double desiredRotation = desiredState.angle.Degrees().value();
  double remainder = std::remainder(desiredRotation - currentRotation, 180);
  double newPos = remainder + currentRotation;
  double minDist = std::abs(std::remainder(newPos - desiredRotation, 180));

  double speedMulti = minDist < 0.001 ? 1 : -1;

  return {desiredState.speed * speedMulti,
          frc::Rotation2d(units::degree_t{newPos})};
}

frc::Rotation2d SwerveModule::GetRotation() {
  return units::degree_t{m_steerMotor.GetPosition()};
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() {
  return units::degree_t{m_steerEncoder.GetAbsolutePosition()};
}

void SwerveModule::SetOffset(double offset) {
  m_angleOffset = units::degree_t{offset};
  SyncEncoders();
}

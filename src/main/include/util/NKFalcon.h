// // Copyright (c) FRC Team 122. All Rights Reserved.

// #pragma once

// #include "ctre/phoenix6/core/CoreTalonFX.hpp"
// #include <ctre/phoenix6/TalonFX.hpp>
// #include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
// #include <ctre/phoenix/sensors/SensorInitializationStrategy.h>

// namespace ctre::phoenix6 {

// class NKFalcon {
// public:
//   NKFalcon(int CANId);

//   /**
//    * Sets position conversion factor (default units rotations)
//    */
//   void SetPositionConversionFactor(double conversion);

//   /**
//    * Sets position conversion factor (default units rotations per minute)
//    */
//   void SetVelocityConversionFactor(double conversion);

//   /**
//    * Resets all parameters to factory defaults.
//    */
//   void ConfigFactoryDefault();

//   /**
//    * Applies all configured settings to the motor, call after setting all
//    * desired parameters.
//    */
//   void ConfigAllSettings();

//   void SetInverted(bool inverted);

//   /**
//    * Feedback device for a particular PID loop.
//    * Note the FeedbackDevice enum holds all possible sensor types. Consult
//    * product documentation to confirm what is available. Alternatively the
//    * product specific enum can be used instead.
//    * @code configs.primaryPID.selectedFeedbackSensor =
//    * (FeedbackDevice)TalonSRXFeedbackDevice::QuadEncoder;
//    * configs.primaryPID.selectedFeedbackSensor =
//    * (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
//    * @endcode
//    */
//   void SetSelectedFeedbackSensor(motorcontrol::FeedbackDevice device);

//   /**
//    * Feedback coefficient of selected sensor.
//    */
//   void SetSelectedFeedbackCoefficient(double coeff);

//   /**
//    * P Gain
//    * This is multiplied by closed loop error in sensor units.
//    * Note the closed loop output interprets a final value of 1023 as full
//    * output. So use a gain of '0.25' to get full output if err is 4096u (Mag
//    * Encoder 1 rotation)
//    *
//    * I Gain
//    * This is multiplied by accumulated closed loop error in sensor units every
//    * PID Loop. Note the closed loop output interprets a final value of 1023 as
//    * full output. So use a gain of '0.00025' to get full output if err is 4096u
//    * for 1000 loops (accumulater holds 4,096,000), [which is equivalent to one
//    * CTRE mag encoder rotation for 1000 milliseconds].
//    *
//    * D Gain
//    * This is multiplied by derivative error (sensor units per PID loop,
//    * typically 1ms). Note the closed loop output interprets a final value of
//    * 1023 as full output. So use a gain of '250' to get full output if derr is
//    * 4096u (Mag Encoder 1 rotation) per 1000 loops (typ 1 sec)
//    *
//    * F Gain
//    * See documentation for calculation details.
//    * If using velocity, motion magic, or motion profile,
//    * use (1023 * duty-cycle / sensor-velocity-sensor-units-per-100ms).
//    */
//   void SetPIDF(double p, double i, double d, double f);

//   /**
//    * Sets the mode of operation during neutral throttle output.
//    * @param neutralMode The desired mode of operation when the Controller output
//    * throttle is neutral (ie brake/coast)
//    */
//   void SetNeutralMode(motorcontrol::NeutralMode neutralMode);

//   /**
//    * Supply-side current limiting. This is typically used to prevent breakers
//    * from tripping.
//    */
//   void SetSupplyCurrentLimit(
//       motorcontrol::SupplyCurrentLimitConfiguration currentConfig);

//   /**
//    * The sensor initialization strategy to use. This will impact the behavior
//    * the next time device boots up. Pick the strategy on how to initialize the
//    * "Position" register. Depending on the mechanism, it may be desirable to
//    * auto set the Position register to match the Absolute Position(swerve for
//    * example). Or it may be desired to zero the sensor on boot(drivetrain
//    * translation sensor or a relative servo). TIP: Tuner's self-test feature
//    * will report what the boot sensor value will be in the event the device is
//    * reset.
//    */
//   void SetSensorInitializationStrategy(
//       sensors::SensorInitializationStrategy strategy);

//   /**
//    * This is the max voltage to apply to the hbridge when voltage compensation
//    * is enabled. For example, if 10 (volts) is specified and a TalonSRX is
//    * commanded to 0.5 (PercentOutput, closed-loop, etc) then the TalonSRX will
//    * attempt to apply a duty-cycle to produce 5V.
//    */
//   void SetVoltageCompensation(double voltage);

//   /**
//    * Seconds to go from 0 to full in open loop/
//    */
//   void SetOpenLoopRampRate(double rampRate);

//   /**
//    * Seconds to go from 0 to full in closed loop
//    */
//   void SetClosedLoopRampRate(double rampRate);

//   /**
//    * Sets the sensor position to the given value.
//    * @param sensorPos Position to set for the selected sensor (in raw sensor
//    * units).
//    * @param pidIdx 0 for Primary closed-loop. 1 for auxiliary closed-loop.
//    * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
//    * config success and report an error if it times out. If zero, no blocking or
//    * checking is performed.
//    * @return Error Code generated by function. 0 indicates no error.
//    */
//   void SetSelectedSensorPosition(double sensorPos, int pidIdx = 0,
//                                  int timeoutMs = 50);

//   double GetPosition();

//   double GetVelocity();

//   void Set(motorcontrol::ControlMode mode, double value);
//   void Set(motorcontrol::ControlMode mode, double demand0,
//            motorcontrol::DemandType demand1Type, double demand1);

//   /**
//    * Desired period for velocity measurement.
//    */
//   void SetVelocityMeasurementPeriod(
//       sensors::SensorVelocityMeasPeriod measurementPeriod);

//   void ConfigSelectedFeedbackSensor(motorcontrol::FeedbackDevice device,
//                                     int pidIdx = 0, int timeoutMs = 0);

//   bool HasResetOccured();

// private:
//   int m_id;
//   double m_positionConversionFactor;
//   double m_velocityConversionFactor;
//   configs::TalonFXConfiguration m_config{};
//   hardware::TalonFX m_motor;
// };

// } // namespace ctre::phoenix

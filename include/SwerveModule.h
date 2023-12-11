#pragma once

#include <string>
#include <units/velocity.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

#include "AbsoluteEncoder.h"

// Class for each swerve module on the robot
class SwerveModule {
  public:
    SwerveModule( const int turnMotorChannel, const int driveMotorChannel, const int absoluteEncoderChannel, const units::degree_t absoluteEncoderOffset );

    void SetDesiredState( const frc::SwerveModuleState& state );

    frc::SwerveModuleState GetState( void );
    frc::SwerveModulePosition GetPosition ( void );

  private:
    std::string m_name;

    ctre::phoenix::motorcontrol::can::TalonFX m_driveMotor;
    ctre::phoenix::motorcontrol::can::TalonFX m_turnMotor;

    ctre::phoenix::sensors::CANCoder m_turnAbsEncoder;

    

      // Use a software PID controller and feedforward for the turn motor.
    frc2::PIDController m_turnPIDController{ 0, 0, 0 };
    frc::SimpleMotorFeedforward<units::degrees> m_turnFF;

    frc::SwerveModuleState state;
    units::degrees_per_second_t speed;
    units::degrees_per_second_t opSpeed;
    units::degree_t dTheta;

    double pidOutput;

    friend class SwerveDrive;
};
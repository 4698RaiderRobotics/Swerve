
#include <numbers>
#include <cmath>
#include <units/angle.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveConstants.h"
#include "SwerveModule.h"

#include "DataLogger.h"

using namespace swerve;

SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const units::degree_t absoluteEncoderOffset ) 
                           : m_driveMotor{ driveMotorChannel },
                           m_turnMotor{ turnMotorChannel },
                           m_turnAbsEncoder{ absoluteEncoderChannel } 
{
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration config; // Config for TalonFX

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered

    m_driveMotor.ConfigAllSettings(config);
    m_driveMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);

    m_driveMotor.Config_kP(0, pidf::kDriveP);
    m_driveMotor.Config_kF(0, pidf::kDriveFF);
    m_driveMotor.Config_kI(0, pidf::kDriveI);
    m_driveMotor.Config_kD(0, pidf::kDriveD);


    m_turnAbsEncoder.SetPosition( m_turnAbsEncoder.GetAbsolutePosition() - absoluteEncoderOffset);

    m_name = fmt::format( "/Swerve Module({}-{})", turnMotorChannel, driveMotorChannel );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    units::degree_t turn_angle = units::degree_t{m_turnAbsEncoder.GetPosition()};

    state = frc::SwerveModuleState::Optimize( referenceState, turn_angle );

    // The setpoint rpm for the motor
    speed = state.speed / swerve::physical::kDriveMetersPerRotation / 60_s * 360_deg;

    // Distance between the setpoint angle and the current angle in degrees
    dTheta =  state.angle.Degrees() - turn_angle;

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^4 of (the error angle) * the rpm
    opSpeed = speed * std::pow( units::math::cos( dTheta ).value(), 4 ); 

    m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, opSpeed.value());

    // Use the software PID and FF to move to the correct angle.
    pidOutput = m_turnPIDController.Calculate( turn_angle.value(), state.angle.Degrees().value() );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    units::meters_per_second_t velocity = m_driveMotor.GetSelectedSensorVelocity() / 2048.0 / 10.0 * 1_tr / 1_s;
    return { velocity, units::degree_t{m_turnAbsEncoder.GetPosition()} };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { units::turn_t{ m_driveMotor.GetSelectedSensorPosition() / 2048.0 } * swerve::physical::kDriveMetersPerRotation, 
             units::degree_t{m_turnAbsEncoder.GetPosition()} };
}

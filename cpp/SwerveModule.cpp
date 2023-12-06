
#include <numbers>
#include <cmath>
#include <units/angle.h>

#include <frc/geometry/Rotation2d.h>

#include "SwerveConstants.h"
#include "SwerveModule.h"

#include "DataLogger.h"

using namespace swerve;

SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset ) 
                           : m_driveMotor{ driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnMotor{ turnMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnAbsEncoder{ absoluteEncoderChannel, absoluteEncoderOffset } 
{
    m_driveMotor.RestoreFactoryDefaults();
    m_driveMotor.SetSmartCurrentLimit( 30 );

    m_drivePIDController.SetP( pidf::kDriveP );
    m_drivePIDController.SetI( pidf::kDriveI );
    m_drivePIDController.SetD( pidf::kDriveD );
    m_drivePIDController.SetFF( pidf::kDriveFF );

    m_turnMotor.RestoreFactoryDefaults();
    m_turnMotor.SetSmartCurrentLimit( 30 );

    m_turnPIDController.SetP( pidf::kTurnP );
    m_turnPIDController.SetI( pidf::kTurnI );
    m_turnPIDController.SetD( pidf::kTurnD );
    m_turnPIDController.EnableContinuousInput( -180, 180 );

    m_name = fmt::format( "/Swerve Module({}-{})", turnMotorChannel, driveMotorChannel );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    units::degree_t turn_angle = m_turnAbsEncoder.GetPosition();

    const auto state = frc::SwerveModuleState::Optimize( referenceState, turn_angle );

    // The setpoint rpm for the motor
    units::revolutions_per_minute_t rpm = state.speed / swerve::physical::kDriveMetersPerRotation;

    // Distance between the setpoint angle and the current angle in degrees
    units::degree_t dTheta =  state.angle.Degrees() - turn_angle;

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^4 of (the error angle) * the rpm
    units::revolutions_per_minute_t opRPM = rpm * std::pow( units::math::cos( dTheta ).value(), 4 ); 
    
    // Use the onboard PID controller to set the motor to the correct velocity.
    m_drivePIDController.SetReference( opRPM.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );

    // Use the software PID and FF to move to the correct angle.
    double pidOutput = m_turnPIDController.Calculate( turn_angle.value(), state.angle.Degrees().value() );
    m_turnMotor.Set( pidOutput );

    DataLogger::GetInstance().SendNT( m_name + "/Turn Setpoint", state.angle.Degrees().value() );
    DataLogger::GetInstance().SendNT( m_name + "/Turn Position", m_turnAbsEncoder.GetPosition().value() );
    DataLogger::GetInstance().SendNT( m_name + "/Turn pidoutput", pidOutput );

    DataLogger::GetInstance().SendNT( m_name + "/Delta Theta", dTheta.value() );
    DataLogger::GetInstance().SendNT( m_name + "/Desired RPM", rpm.value() );
    DataLogger::GetInstance().SendNT( m_name + "/Optimized RPM", opRPM.value() );
    DataLogger::GetInstance().SendNT( m_name + "/Drive Current", m_driveMotor.GetOutputCurrent() );
    DataLogger::GetInstance().SendNT( m_name + "/Turn Current", m_turnMotor.GetOutputCurrent() );
    DataLogger::GetInstance().SendNT( m_name + "/Drive Motor Speed", m_driveEncoder.GetVelocity() );
    DataLogger::GetInstance().SendNT( m_name + "/Turn Motor Speed", m_turnMotorEncoder.GetVelocity() );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    units::meters_per_second_t velocity = m_driveEncoder.GetVelocity() * 1_rpm * physical::kDriveMetersPerRotation * 1_min / 60_s;
    return { velocity, m_turnAbsEncoder.GetPosition() };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { units::turn_t{ m_driveEncoder.GetPosition() } * swerve::physical::kDriveMetersPerRotation, 
             m_turnAbsEncoder.GetPosition()  };
}


#include <numbers>
#include <cmath>
#include <units/angle.h>

#include <frc/geometry/Rotation2d.h>

#include "SwerveConstants.h"
#include "SwerveModule.h"

using namespace swerve;

SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset ) 
                           : m_driveMotor{ driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnMotor{ turnMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless },
                           m_turnAbsEncoder{ absoluteEncoderChannel, absoluteEncoderOffset } 
{
    m_drivePIDController.SetP( pidf::kDriveP );
    m_drivePIDController.SetI( pidf::kDriveI );
    m_drivePIDController.SetD( pidf::kDriveD );
    m_drivePIDController.SetFF( pidf::kDriveFF );

    m_turnPIDController.SetP( pidf::kTurnP );
    m_turnPIDController.SetI( pidf::kTurnI );
    m_turnPIDController.SetD( pidf::kTurnD );
    m_turnPIDController.EnableContinuousInput( -180, 180 );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {

    const auto state = frc::SwerveModuleState::Optimize( referenceState, m_turnAbsEncoder.GetPosition() );

    // The setpoint rpm for the motor
    units::revolutions_per_minute_t rpm = state.speed / swerve::physical::kDriveMetersPerRotation;

    units::degree_t turn_angle = m_turnAbsEncoder.GetPosition();
    // Distance between the setpoint angle and the current angle in degrees
    units::degree_t dTheta =  state.angle.Degrees() - turn_angle;

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^4 of (the error angle) * the rpm
    units::revolutions_per_minute_t opRPM = rpm * std::pow( units::math::cos( dTheta ).value(), 4 ); 
    
    // Use the onboard PID controller to set the motor to the correct velocity.
    m_drivePIDController.SetReference( opRPM.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );

    // Use the software PID and FF to move to the correct angle.
    double feedback = m_turnPIDController.Calculate( turn_angle.value(), state.angle.Degrees().value() );
    m_turnMotor.Set( feedback );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    return { units::meters_per_second_t{ m_driveEncoder.GetVelocity() }, units::radian_t{ m_turnAbsEncoder.GetPosition() } };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { units::turn_t{ m_driveEncoder.GetPosition() } * swerve::physical::kDriveMetersPerRotation, 
             m_turnAbsEncoder.GetPosition()  };
}

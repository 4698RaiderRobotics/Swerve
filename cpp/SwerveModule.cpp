
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
    m_turnPIDController.SetFF( pidf::kTurnFF );
    m_turnPIDController.SetPositionPIDWrappingEnabled( true );
    m_turnPIDController.SetPositionPIDWrappingMaxInput( 0.5 * physical::kTurnGearRatio );
    m_turnPIDController.SetPositionPIDWrappingMaxInput( -0.5 * physical::kTurnGearRatio );

    m_turnPIDController.SetSmartMotionMaxVelocity( pidf::turn_kMaxVel );
    m_turnPIDController.SetSmartMotionMinOutputVelocity( pidf::turn_kMinVel );
    m_turnPIDController.SetSmartMotionMaxAccel( pidf::turn_kMaxAcc );
    m_turnPIDController.SetSmartMotionAllowedClosedLoopError( pidf::turn_kAllErr );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
        // Set the relative encoders position based on the Absolute Encoders reading
    if( !m_turnEncoderSet ) {
        if( m_turnAbsEncoder.IsConnected() ) {
            m_turnEncoder.SetPosition( m_turnAbsEncoder.GetPosition().value() * physical::kTurnGearRatio / 360.0 );
            m_turnEncoderSet = true;
        } else {
            // No encoder signal.
            return;
        }
    }

    const auto state = frc::SwerveModuleState::Optimize( referenceState, m_turnAbsEncoder.GetPosition() );

    // The setpoint rpm for the motor
    units::revolutions_per_minute_t rpm = state.speed / swerve::physical::kDriveMetersPerRotation;

    // Distance between the setpoint angle and the current angle in degrees
    units::degree_t dTheta =  state.angle.Degrees() - m_turnAbsEncoder.GetPosition();

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^4 of (the error angle) * the rpm
    units::revolutions_per_minute_t opRPM = rpm * std::pow( units::math::cos( dTheta ).value(), 4 ); 
    
    // Use the onboard PID controller to set the motor to the correct velocity.
    m_drivePIDController.SetReference( opRPM.value(), rev::CANSparkMaxLowLevel::ControlType::kVelocity );

    // Use the onboard PID controller and Smart Motion to move to the correct angle.
    double rotation_setpt = physical::kTurnGearRatio * state.angle.Degrees().value() / 360.0;
    m_turnPIDController.SetReference( rotation_setpt, rev::CANSparkMaxLowLevel::ControlType::kSmartMotion );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    return { units::meters_per_second_t{ m_driveEncoder.GetVelocity() }, units::radian_t{ m_turnEncoder.GetPosition() } };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { units::turn_t{ m_driveEncoder.GetPosition() } * swerve::physical::kDriveMetersPerRotation, 
             m_turnAbsEncoder.GetPosition()  };
}

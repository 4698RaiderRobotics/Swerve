#pragma once

#include <string>
#include <units/velocity.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

#include "AbsoluteEncoder.h"

// Class for each swerve module on the robot
class SwerveModule {
    public:
        SwerveModule( const int turnMotorChannel, const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset );

        void SetDesiredState( const frc::SwerveModuleState& state );

        frc::SwerveModuleState GetState( void );
        frc::SwerveModulePosition GetPosition ( void );
    private:
        rev::CANSparkMax m_driveMotor;
        rev::CANSparkMax m_turnMotor;

        rev::SparkMaxRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
        AbsoluteEncoder m_turnAbsEncoder;

        // Use the onboard PID controller (rev::SparkMaxPIDController). 
        // for the drive motor.
        rev::SparkMaxPIDController m_drivePIDController = m_driveMotor.GetPIDController();

          // Use a software PID controller and feedforward for the turn motor.
        frc2::PIDController m_turnPIDController{ 0, 0, 0 };
        frc::SimpleMotorFeedforward<units::degrees> m_turnFF;

        friend class SwerveDrive;
};
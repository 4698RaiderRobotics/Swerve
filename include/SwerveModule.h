#pragma once

#include <string>
#include <units/velocity.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>

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
        rev::SparkMaxRelativeEncoder m_turnEncoder = m_turnMotor.GetEncoder();
        AbsoluteEncoder m_turnAbsEncoder;
          // Has the relative encoder been set to the absolute encoders position?
        bool m_turnEncoderSet{ false };

        // Use the onboard PID controllers (rev::SparkMaxPIDController). 
        // The motor is automatically set by the PID controller.
        rev::SparkMaxPIDController m_drivePIDController = m_driveMotor.GetPIDController();
        rev::SparkMaxPIDController m_turnPIDController = m_turnMotor.GetPIDController();

        friend class SwerveDrive;
};
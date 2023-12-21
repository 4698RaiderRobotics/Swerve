/*
 * 
 *      Team 4698 Raider Robotics Swerve Drive Code
 * 
 *      First Created 2022
 *      by S.Smith, T.Smith
 * 
*/

#include "SwerveConstants.h"
#include "SwerveDrive.h"
#include "DataLogger.h"

#ifdef TUNING
#include <frc/smartdashboard/SmartDashboard.h>
#endif /* TUNING */

using namespace swerve;

SwerveDrive::SwerveDrive( ) 
    : m_modules{ SwerveModule{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
                               deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset },
                 SwerveModule{ deviceIDs::kFrontRightTurnMotorID, deviceIDs::kFrontRightDriveMotorID, 
                               deviceIDs::kFrontRightAbsoluteEncoderID, physical::kFrontRightAbsoluteOffset },
                 SwerveModule{ deviceIDs::kBackLeftTurnMotorID, deviceIDs::kBackLeftDriveMotorID, 
                               deviceIDs::kBackLeftAbsoluteEncoderID, physical::kBackLeftAbsoluteOffset },
                 SwerveModule{ deviceIDs::kBackRightTurnMotorID, deviceIDs::kBackRightDriveMotorID, 
                               deviceIDs::kBackRightAbsoluteEncoderID, physical::kBackRightAbsoluteOffset } }
    , m_gyro{deviceIDs::kPigeonIMUID} 
    , m_kinematics{ frc::Translation2d{+( physical::kDriveBaseLength / 2 ), +( physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{+( physical::kDriveBaseLength / 2 ), -( physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( physical::kDriveBaseLength / 2 ), +( physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( physical::kDriveBaseLength / 2 ), -( physical::kDriveBaseWidth / 2 )} }
    , m_odometry{ m_kinematics, frc::Rotation2d{ 0_deg },
                    { m_modules[0].GetPosition(), 
                      m_modules[1].GetPosition(),
                      m_modules[2].GetPosition(), 
                      m_modules[3].GetPosition()
                    }, frc::Pose2d{ 0_ft, 0_ft, 0_deg } }
    , m_controller{ frc::PIDController{ pidf::X_Holo_kP, pidf::X_Holo_kI, pidf::X_Holo_kD }, 
                    frc::PIDController{ pidf::Y_Holo_kP, pidf::Y_Holo_kI, pidf::Y_Holo_kD },
                    frc::ProfiledPIDController<units::radian> {
                        pidf::Th_Holo_kP, pidf::Th_Holo_kI, pidf::Th_Holo_kD, 
                        frc::TrapezoidProfile<units::radian>::Constraints{
                            pidf::Th_Holo_MaxVel, pidf::Th_Holo_MaxAcc }}}

{
#ifdef TUNING
        // Holonomic Controller parameters
    frc::SmartDashboard::PutNumber("X_Holo P", m_controller.getXController().GetP() );
    frc::SmartDashboard::PutNumber("X_Holo I", m_controller.getXController().GetI() );
    frc::SmartDashboard::PutNumber("X_Holo D", m_controller.getXController().GetD() );
    frc::SmartDashboard::PutNumber("Y_Holo P", m_controller.getYController().GetP() );
    frc::SmartDashboard::PutNumber("Y_Holo I", m_controller.getYController().GetI() );
    frc::SmartDashboard::PutNumber("Y_Holo D", m_controller.getYController().GetD() );
    frc::SmartDashboard::PutNumber("Th_Holo P", m_controller.getThetaController().GetP() );
    frc::SmartDashboard::PutNumber("Th_Holo I", m_controller.getThetaController().GetI() );
    frc::SmartDashboard::PutNumber("Th_Holo D", m_controller.getThetaController().GetD() );
    frc::SmartDashboard::PutNumber("Th_Holo MaxVel", pidf::Th_Holo_MaxVel.value() );
    frc::SmartDashboard::PutNumber("Th_Holo MaxAcc", pidf::Th_Holo_MaxAcc.value() );
    
        // Swerve Module parameters
    // frc::SmartDashboard::PutNumber("Drive P", m_modules[0].m_drivePIDController.GetP() );
    // frc::SmartDashboard::PutNumber("Drive I", m_modules[0].m_drivePIDController.GetI() );
    // frc::SmartDashboard::PutNumber("Drive D", m_modules[0].m_drivePIDController.GetD() );
    // frc::SmartDashboard::PutNumber("Drive FF", m_modules[0].m_drivePIDController.GetFF() );
    frc::SmartDashboard::PutNumber("Turn P", m_modules[0].m_turnPIDController.GetP() );
    frc::SmartDashboard::PutNumber("Turn I", m_modules[0].m_turnPIDController.GetI() );
    frc::SmartDashboard::PutNumber("Turn D", m_modules[0].m_turnPIDController.GetD() );

    frc::SmartDashboard::PutBoolean("Update Parameters", false );
#endif /* TUNING */

        // Reset the gyro
    ResetGyro(0_deg);
}

// ArcadeDrive drives with joystick inputs
// This takes -1 to 1 inputs
void SwerveDrive::ArcadeDrive( double xPercent, double yPercent, double omegaPercent, bool fieldRelative ) {
    auto x = xPercent * physical::kMaxDriveSpeed;
    auto y = yPercent * physical::kMaxDriveSpeed;
    auto omega = omegaPercent * physical::kMaxTurnSpeed;

    frc::ChassisSpeeds speeds{ x, y, omega };
    Drive( speeds, fieldRelative );
}

void SwerveDrive::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    // An array of SwerveModuleStates computed from the ChassisSpeeds object
    m_desiredStates = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
                    speeds.vx, speeds.vy, speeds.omega, frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } } ) :
                    speeds );
    m_kinematics.DesaturateWheelSpeeds( &m_desiredStates, swerve::physical::kMaxDriveSpeed );
}

// Drives a path given a trajectory state
void SwerveDrive::DriveTrajectory( frc::Trajectory::State trajState, const frc::Rotation2d &robotHeading ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetPose(), trajState, robotHeading.Degrees() );

    Drive( adjustedSpeeds );
}

void SwerveDrive::Periodic( void ) {

#ifdef TUNING
    if( frc::SmartDashboard::GetBoolean("Update Parameters", false ) ) {
        TuneSwerveDrive();
        frc::SmartDashboard::PutBoolean("Update Parameters", false );
    }

    frc::SmartDashboard::PutNumber("Front Left Absolute Position", m_modules[0].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Front Right Absolute Position", m_modules[1].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Back Left Absolute Position", m_modules[2].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Back Right Absolute Position", m_modules[3].m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble());

    frc::SmartDashboard::PutNumber("Turn Motor Position", m_modules[0].GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Turn Motor Position Setpoint", m_desiredStates[0].angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Drive Motor Velocity", m_modules[0].GetState().speed.value());
    frc::SmartDashboard::PutNumber("Drive Motor Velocity Setpoint", m_desiredStates[0].speed.value());

    m_swerve_display.SetState( m_desiredStates );

#else
    for(int i = 0; i < 3; i++) {
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Setpoint", m_modules[i].state.angle.Degrees().value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Position", m_modules[i].m_turnAbsEncoder.GetPosition() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Raw Position", m_modules[i].m_turnAbsEncoder.GetAbsolutePosition() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn pidoutput", m_modules[i].pidOutput );

        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Delta Theta", m_modules[i].dTheta.value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Desired RPM", m_modules[i].rpm.value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Optimized RPM", m_modules[i].opRPM.value() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Drive Current", m_modules[i].m_driveMotor.GetOutputCurrent() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Current", m_modules[i].m_turnMotor.GetOutputCurrent() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Drive Motor Speed", m_modules[i].m_driveEncoder.GetVelocity() );
        DataLogger::GetInstance().SendNT( m_modules[i].m_name + "/Turn Motor Speed", m_modules[i].m_turnMotorEncoder.GetVelocity() );
    }

#endif /* TUNING */

    // Sets each SwerveModule to the correct SwerveModuleState
    for( int i=0; i<4; ++i ) {
        m_modules[i].SetDesiredState( m_desiredStates[i] );
    }

    // Updates the odometry of the robot given the SwerveModules' states
    //needs to be an array
    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
    {
         m_modules[0].GetPosition(),  m_modules[1].GetPosition(), 
         m_modules[2].GetPosition(),  m_modules[3].GetPosition() 
    });

    if( m_logging ) {
        // Log the swerve states
        for( int i=0; i<4; ++i ) {
            m_actualStates[i] = m_modules[i].GetState();
        }
        LogSwerveStateArray( m_actualLogEntry, m_actualStates );
        LogSwerveStateArray( m_desiredLogEntry, m_desiredStates );

        // Log the gyro angle
        m_gyroYawLogEntry.Append( m_gyro.GetYaw() );

        // Log the Robot pose
        frc::Pose2d currentPose = m_odometry.GetPose();
        m_poseLogEntry.Append( { currentPose.X().value(), 
                                 currentPose.Y().value(), 
                                 currentPose.Rotation().Degrees().value() } );
    }
}

// Returns the pose2d of the robot
frc::Pose2d SwerveDrive::GetPose( void ) {
    return m_odometry.GetPose();
}

// Resets the gyro to an angle
void SwerveDrive::ResetGyro( units::degree_t angle ) {
    m_gyro.SetYaw( angle.value() );
}

// Resets the pose to a position
void SwerveDrive::ResetPose( frc::Translation2d position ) {
    m_odometry.ResetPosition(
        frc::Rotation2d{   units::degree_t{ m_gyro.GetYaw() }  },
        {
            m_modules[0].GetPosition(),  m_modules[1].GetPosition(), 
            m_modules[2].GetPosition(),  m_modules[3].GetPosition() 
        },
        frc::Pose2d{ position.X(), position.Y(), units::degree_t{ m_gyro.GetYaw() } }
    );
}

void SwerveDrive::StartLogging( wpi::log::DataLog& log ) {
    m_logging = true;
    m_actualLogEntry = wpi::log::DoubleArrayLogEntry( log, "Swerve/Actual States" );
    m_desiredLogEntry = wpi::log::DoubleArrayLogEntry( log, "Swerve/Desired States" );
    m_poseLogEntry = wpi::log::DoubleArrayLogEntry( log, "Robot/Robot2D" );
    m_gyroYawLogEntry = wpi::log::DoubleLogEntry( log, "Swerve/GyroYaw" );
}

void SwerveDrive::LogSwerveStateArray( wpi::log::DoubleArrayLogEntry& logEntry, 
                                       wpi::array<frc::SwerveModuleState, 4U> states ) {
    static double state_array[8];

    for( int i=0; i<4; ++i ) {
        state_array[2*i] = states[i].angle.Radians().value(); 
        state_array[2*i + 1] = states[i].speed.value();
    }
    logEntry.Append( state_array );
}

void SwerveDrive::TuneSwerveDrive() {
#ifdef TUNING
    double val;
    static units::radians_per_second_t MaxVel{ pidf::Th_Holo_MaxVel };
    static units::radians_per_second_squared_t MaxAcc{ pidf::Th_Holo_MaxAcc };

#define SET_HOLO_IF_CHANGED( name, pidc, getf, setf ) \
            val = frc::SmartDashboard::GetNumber((name), pidc.getf() ); \
            if( val != pidc.getf() ) { pidc.setf( val ); }

    SET_HOLO_IF_CHANGED( "X_Holo P", m_controller.getXController(), GetP, SetP )
    SET_HOLO_IF_CHANGED( "X_Holo I", m_controller.getXController(), GetI, SetI )
    SET_HOLO_IF_CHANGED( "X_Holo D", m_controller.getXController(), GetD, SetD )
    SET_HOLO_IF_CHANGED( "Y_Holo P", m_controller.getYController(), GetP, SetP )
    SET_HOLO_IF_CHANGED( "Y_Holo I", m_controller.getYController(), GetI, SetI )
    SET_HOLO_IF_CHANGED( "Y_Holo D", m_controller.getYController(), GetD, SetD )
    SET_HOLO_IF_CHANGED( "Th_Holo P", m_controller.getThetaController(), GetP, SetP )
    SET_HOLO_IF_CHANGED( "Th_Holo I", m_controller.getThetaController(), GetI, SetI )
    SET_HOLO_IF_CHANGED( "Th_Holo D", m_controller.getThetaController(), GetD, SetD )

    double sd_maxVel = frc::SmartDashboard::GetNumber( "Th_Holo MaxVel", pidf::Th_Holo_MaxVel.value() );
    double sd_maxAcc = frc::SmartDashboard::GetNumber( "Th_Holo MaxAcc", pidf::Th_Holo_MaxAcc.value() );
    if( sd_maxVel != MaxVel.value() || sd_maxAcc != MaxAcc.value() ) {
        MaxVel = units::radians_per_second_t{ sd_maxVel };
        MaxAcc = units::radians_per_second_squared_t{ sd_maxAcc };
        m_controller.getThetaController().SetConstraints( { MaxVel, MaxAcc } );
    }

#define SET_MODULES_IF_CHANGED( name, mods, pidc, getf, setf ) \
            val = frc::SmartDashboard::GetNumber((name), mods[0].pidc.getf() ); \
            if( val != mods[0].pidc.getf() ) { \
                for(int i=0; i<4; ++i ) { \
                    mods[i].pidc.setf( val ); \
                } \
            }

    // SET_MODULES_IF_CHANGED( "Drive P", m_modules, m_drivePIDController, GetP, SetP )
    // SET_MODULES_IF_CHANGED( "Drive I", m_modules, m_drivePIDController, GetI, SetI )
    // SET_MODULES_IF_CHANGED( "Drive D", m_modules, m_drivePIDController, GetD, SetD )
    // SET_MODULES_IF_CHANGED( "Drive FF", m_modules, m_drivePIDController, GetFF, SetFF )
    SET_MODULES_IF_CHANGED( "Turn P", m_modules, m_turnPIDController, GetP, SetP )
    SET_MODULES_IF_CHANGED( "Turn I", m_modules, m_turnPIDController, GetI, SetI )
    SET_MODULES_IF_CHANGED( "Turn D", m_modules, m_turnPIDController, GetD, SetD )
#endif /* TUNING */
}
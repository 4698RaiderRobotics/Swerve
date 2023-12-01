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

using namespace swerve;

SwerveDrive::SwerveDrive( ) 
    : m_gyro{deviceIDs::kPigeonIMUID} 
    , m_modules{ SwerveModule{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
                               deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset },
                 SwerveModule{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
                               deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset },
                 SwerveModule{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
                               deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset },
                 SwerveModule{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
                               deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset } }
    ,  m_kinematics{ frc::Translation2d{+( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 )},
                     frc::Translation2d{+( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 )},
                     frc::Translation2d{-( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 )},
                     frc::Translation2d{-( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 )} }
    , m_odometry{ m_kinematics, frc::Rotation2d{ 0_deg },
                    { m_modules[0].GetPosition(), 
                      m_modules[1].GetPosition(),
                      m_modules[2].GetPosition(), 
                      m_modules[3].GetPosition()
                    }, frc::Pose2d{ 0_ft, 0_ft, 0_deg } }
    , m_controller{ frc2::PIDController{ 1, 0, 0 }, frc2::PIDController{ 1, 0, 0 },
                    frc::ProfiledPIDController<units::radian> {
                    1, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{
                    6.28_rad_per_s, 3.14_rad_per_s / 1_s}}}

{}

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
void SwerveDrive::ResetGyro( int angle ) {
    m_gyro.SetYaw( angle );
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

/*
 * 
 *      Team 4698 Raider Robotics Swerve Drive Code
 * 
 *      First Created 2022
 *      by S.Smith, T.Smith
 * 
*/

#include "SwerveDrive.h"


void SwerveDrive::Drive( frc::ChassisSpeeds speeds, bool fieldRelative ) {
    // An array of SwerveModuleStates computed from the ChassisSpeeds object
    m_desiredStates = m_kinematics.ToSwerveModuleStates( fieldRelative ? speeds.FromFieldRelativeSpeeds( 
                    speeds.vx, speeds.vy, speeds.omega, frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } } ) :
                    speeds );
    m_kinematics.DesaturateWheelSpeeds( &m_desiredStates, physical::kMaxDriveSpeed );
}

// Drives a path given a trajectory state
void SwerveDrive::DriveTrajectory( frc::Trajectory::State trajectoryState ) {
    // A ChassisSpeeds objects based on the current position on the trajectory
    auto adjustedSpeeds = m_controller.Calculate( m_odometry.GetPose(), trajectoryState, trajectoryState.pose.Rotation().Degrees() );

    Drive( adjustedSpeeds );
}

void SwerveDrive::Periodic( void ) {
    // Sets each SwerveModule to the correct SwerveModuleState
    m_frontLeft.SetDesiredState( m_desiredStates[0] );
    m_frontRight.SetDesiredState( m_desiredStates[1] );
    m_backLeft.SetDesiredState( m_desiredStates[2] );
    m_backRight.SetDesiredState( m_desiredStates[3] );

    // Updates the odometry of the robot given the SwerveModules' states
    //needs to be an array
    m_odometry.Update( frc::Rotation2d{ units::degree_t{ m_gyro.GetYaw() } },
    {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
        m_backLeft.GetPosition(), m_backRight.GetPosition() 
    });

    if( m_logging ) {
        // Log the swerve states
        m_actualStates[0] = m_frontLeft.GetState();
        m_actualStates[1] = m_frontRight.GetState();
        m_actualStates[2] = m_backLeft.GetState();
        m_actualStates[3] = m_backRight.GetState();
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
            m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
            m_backLeft.GetPosition(), m_backRight.GetPosition() 
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

void SwerveDrive::LogSwerveStateArray( wpi::log::DoubleArrayLogEntry& logEntry, wpi::array<frc::SwerveModuleState, 4U> states ) {
    static double state_array[8];

    for( int i=0; i<4; ++i ) {
        state_array[2*i] = states[i].angle.Degrees().value(); 
        state_array[2*i + 1] = states[i].speed.value();
    }
    logEntry.Append( state_array );
}

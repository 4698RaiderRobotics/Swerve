/*
 * 
 *      Team 4698 Raider Robotics Swerve Drive Code
 * 
 *      First Created 2022
 *      by S.Smith, T.Smith
 * 
*/

#pragma once

#include <wpi/array.h>
#include <wpi/DataLog.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/HolonomicDriveController.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <frc/geometry/Pose2d.h>
#include <units/time.h>
using namespace units::literals;

#include "SwerveModule.h"
#include "SwerveModuleDisplay.h"

class SwerveDrive {
  public:
    SwerveDrive( );
    
    void ArcadeDrive( double xPercent, double yPercent, double omegaPercent, bool fieldRelative = true );
    void Drive( frc::ChassisSpeeds speeds, bool fieldRelative = true );

    void DriveTrajectory( frc::Trajectory::State trajState, const frc::Rotation2d &robotHeading );

    void Periodic( void );

    frc::Pose2d GetPose( void );

    void ResetGyro( int angle );

    void ResetPose( frc::Translation2d position );

    void StartLogging( wpi::log::DataLog& log );

  private:
    void LogSwerveStateArray(  wpi::log::DoubleArrayLogEntry& logEntry, wpi::array<frc::SwerveModuleState, 4U> states );
    void TuneSwerveDrive();

  //  SwerveStatusDisplay swerve_display{ "Swerve Drive", "Robot Wheel Status" };

    // SwerveModule m_frontLeft{ deviceIDs::kFrontLeftTurnMotorID, deviceIDs::kFrontLeftDriveMotorID, 
    //                         deviceIDs::kFrontLeftAbsoluteEncoderID, physical::kFrontLeftAbsoluteOffset };
    // SwerveModule m_frontRight{ deviceIDs::kFrontRightTurnMotorID, deviceIDs::kFrontRightDriveMotorID, 
    //                         deviceIDs::kFrontRightAbsoluteEncoderID, physical::kFrontRightAbsoluteOffset };
    // SwerveModule m_backLeft{ deviceIDs::kBackLeftTurnMotorID, deviceIDs::kBackLeftDriveMotorID, 
    //                         deviceIDs::kBackLeftAbsoluteEncoderID, physical::kBackLeftAbsoluteOffset };
    // SwerveModule m_backRight{ deviceIDs::kBackRightTurnMotorID, deviceIDs::kBackRightDriveMotorID, 
    //                         deviceIDs::kBackRightAbsoluteEncoderID, physical::kBackRightAbsoluteOffset };
    SwerveModule m_modules[4];

    frc::Trajectory m_trajectory;
    wpi::array<frc::SwerveModuleState, 4U> m_desiredStates{ wpi::empty_array };
    wpi::array<frc::SwerveModuleState, 4U> m_actualStates{ wpi::empty_array };

    ctre::phoenix::sensors::PigeonIMU m_gyro;

    // ctre::phoenix::sensors::PigeonIMU m_gyro{deviceIDs::kPigeonIMUID};

    // frc::Translation2d m_frontLeftLocation{ +( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 ) };
    // frc::Translation2d m_frontRightLocation{ +( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 ) };
    // frc::Translation2d m_backLeftLocation{ -( physical::kDriveBaseWidth / 2 ), +( physical::kDriveBaseWidth / 2 ) };
    // frc::Translation2d m_backRightLocation{ -( physical::kDriveBaseWidth / 2 ), -( physical::kDriveBaseWidth / 2 ) };

    frc::Translation2d m_mod_Location[4];

    // frc::SwerveDriveKinematics<4> m_kinematics{ m_frontLeftLocation, m_frontRightLocation, 
    //                                           m_backLeftLocation,m_backRightLocation};
    // frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, frc::Rotation2d{ 0_deg },
    //   {
    //     m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
    //     m_backLeft.GetPosition(), m_backRight.GetPosition()
    //   },
    //   frc::Pose2d{ 0_ft, 0_ft, 0_deg } 
    // };

    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDriveOdometry<4> m_odometry;

    // Drive controller for driving a trajectory
    frc::HolonomicDriveController m_controller;

    // frc::HolonomicDriveController m_controller{ 
    //       frc2::PIDController{ 1, 0, 0 }, frc2::PIDController{ 1, 0, 0 },
    //       frc::ProfiledPIDController<units::radian> {
    //         1, 0, 0, frc::TrapezoidProfile<units::radian>::Constraints{
    //           6.28_rad_per_s, 3.14_rad_per_s / 1_s}}};


    wpi::log::DoubleArrayLogEntry m_actualLogEntry;
    wpi::log::DoubleArrayLogEntry m_desiredLogEntry;
    wpi::log::DoubleArrayLogEntry m_poseLogEntry;
    wpi::log::DoubleLogEntry m_gyroYawLogEntry;
    bool m_logging{ false };
};
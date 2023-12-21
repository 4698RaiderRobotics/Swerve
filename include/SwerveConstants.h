#pragma once

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

// Turn on some tuning stuff.
#define TUNING

namespace swerve {
    namespace pidf {
        constexpr double kTurnP = 5.0;
        constexpr double kTurnI = 0.0;
        constexpr double kTurnD = 0.0;

        constexpr double kDriveP = 0.1;
        constexpr double kDriveI = 0.0;
        constexpr double kDriveD = 0.0;
        constexpr double kDriveV = 0.113;

            // Holonomic Controller Constants
        constexpr double X_Holo_kP = 1;
        constexpr double X_Holo_kI = 0;
        constexpr double X_Holo_kD = 0;

        constexpr double Y_Holo_kP = 1;
        constexpr double Y_Holo_kI = 0;
        constexpr double Y_Holo_kD = 0;

        constexpr double Th_Holo_kP = 1;
        constexpr double Th_Holo_kI = 0;
        constexpr double Th_Holo_kD = 0;

        constexpr units::radians_per_second_t Th_Holo_MaxVel = 6.28_rad_per_s;
        constexpr units::radians_per_second_squared_t Th_Holo_MaxAcc = 3.14_rad_per_s_sq;
    }

    namespace deviceIDs {
        constexpr int kFrontLeftTurnMotorID = 1;
        constexpr int kFrontLeftDriveMotorID = 2;
        constexpr int kFrontRightTurnMotorID = 3;
        constexpr int kFrontRightDriveMotorID = 4;
        constexpr int kBackLeftTurnMotorID = 5;
        constexpr int kBackLeftDriveMotorID = 6;
        constexpr int kBackRightTurnMotorID = 7;
        constexpr int kBackRightDriveMotorID = 8;

        constexpr int kFrontLeftAbsoluteEncoderID = 9;
        constexpr int kFrontRightAbsoluteEncoderID = 10;
        constexpr int kBackLeftAbsoluteEncoderID = 11;
        constexpr int kBackRightAbsoluteEncoderID = 12;

        const int kPigeonIMUID = 13;
    }

    namespace physical {
        // Max drive speed of Mk3 swerve modules * a scalar value
        constexpr units::meters_per_second_t kMaxDriveSpeed = 15.7_fps;

        // The max speed of the turn motors
        constexpr auto kMaxTurnSpeed =  5_rad_per_s;

        // Gear ratio of the drive motors. 6.86 rotations of the drive motor is one rotation of the wheel.
        constexpr double kDriveGearRatio = 6.75;

        // Compound unit for the meter per revolution constant.
        using meters_per_rev = units::compound_unit<units::meters, units::inverse<units::turns>>;
        typedef units::unit_t<meters_per_rev> meters_per_rev_t;

        // The number of meters traveled per rotation of the drive motor
        // wheel circumference / gear ratio
        constexpr meters_per_rev_t kDriveMetersPerRotation = std::numbers::pi * 4_in / (kDriveGearRatio *  1_tr );

        // Gear ratio of the turn motors. 12.8 rotations of the turning motor is one rotation of the swerve module.
        constexpr double kTurnGearRatio = 150.0 / 7.0;

        // The width of the drive base from the center of one module to another adjacent one.
        constexpr units::meter_t kDriveBaseWidth = 23.25_in * 1.08;
        constexpr units::meter_t kDriveBaseLength = 22.5_in * 1.08;

        constexpr double kFrontLeftAbsoluteOffset = -0.413;
        constexpr double kFrontRightAbsoluteOffset = -0.361;
        constexpr double kBackLeftAbsoluteOffset = 0.102;
        constexpr double kBackRightAbsoluteOffset = 0.180;
    }
}
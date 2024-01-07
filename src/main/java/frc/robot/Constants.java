package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerve.SwerveModuleConstants;

/**
 * Constants file.
 */

public final class Constants {
    public static final double stickDeadband = 0.1;

    /**
     * PID constants for Swerve Auto Holonomic Drive Controller
     */
    public static class SwerveTransformPID {
        public static final double pidXkP = 1.5;
        public static final double pidXkI = 0.0;
        public static final double pidXkD = 0.0;
        public static final double pidYkP = 1.5;
        public static final double pidYkI = 0.0;
        public static final double pidYkD = 0.0;
        public static final double pidTkP = 3.0;
        public static final double pidTkI = 0.0;
        public static final double pidTkD = 0.0;

        public static final double maxAngularVelocity = 3.0;
        public static final double maxAngularAcceleration = 3.0;
        public static final double stdDevMod = 2.0;
    }

    /**
     * Camera offset constants
     */
    public static class CameraConstants {

        public static final double pitch = 0 * Math.PI / 180;
        public static final double roll = Math.PI;
        public static final double yaw = 0.0;
        public static final Transform3d kCameraToRobot =
            new Transform3d(new Translation3d(Units.inchesToMeters(-10), Units.inchesToMeters(2),
                Units.inchesToMeters(7.5)), new Rotation3d(roll, pitch, yaw));
        public static final String cameraName = "pv2";
        public static final double largestDistance = 0.1;
    }

    /**
     * Swerve ID's
     */
    public static final class Swerve {
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(14);
        public static final double wheelBase = Units.inchesToMeters(14);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.4;

        public static final double driveGearRatio = (8.14 / 1.0); // 8.14:1
        public static final double angleGearRatio = ((150 / 7) / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        public static final double angleMotorEncoderPositionPIDMinInput = 0;
        public static final double angleMotorEncoderPositionPIDMaxInput = 360;
        public static final double turningDegreesPerEncoderRevolution = 360 / angleGearRatio;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12);
        // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4; // meters per second
        public static final double maxAngularVelocity = 2;

        /* Neutral Modes */
        // public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;

        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /**
         * Front Left Module - Module 0.
         */
        public static final class Mod0 {
            public static final int driveMotorID = 50;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final double angleOffset = 139.043;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Front Right Module - Module 1. PROBLEM CHILD
         */
        public static final class Mod1 {
            public static final int driveMotorID = 51;
            public static final int angleMotorID = 52;
            public static final int canCoderID = 2;
            public static final double angleOffset = 118.564;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Left Module - Module 2.
         */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 259.805;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Right Module - Module 3.
         */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 53;
            public static final int canCoderID = 4;
            public static final double angleOffset = 31.816;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }



    /**
     * Autonomous constants for swerve bot.
     */
    public static final class AutoConstants {

    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 20;
    public static final int kLeftMotor2Port = 21;
    public static final int kRightMotor1Port = 23;
    public static final int kRightMotor2Port = 24;

    public static final double kTrackwidthMeters = 0.5797;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) (kEncoderCPR*10.71);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.194;
    public static final double kvVoltSecondsPerMeter = 2.79;
    public static final double kaVoltSecondsSquaredPerMeter = 0.381;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 2.28;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 0;
    public static final int kIntakeForwardPort = 6;
    public static final int kIntakeReversePort = 14;
  }

  public static final class ClimberConstants {
    public static final int kLeftMotorPort = 25;
    public static final int kRightMotorPort = 22;
    public static final int kClimberForwardPort = 7;
    public static final int kClimberReversePort = 15;
  }

  public static final class ShooterConstants {
    public static final int kFrontMotorPort = 30;
    public static final int kBackMotorPort = 31;
    public static final int kKickerForwardPort = 5;
    public static final int kKickerReversePort = 13;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverStickPort = 1;
    public static final int kButtonBoxPort = 2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5; //1.5
    public static final double kFastMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
  public static final class LimelightConstants {
    // FIGURE THESE OUT
    public static final double kLimelightHeight = 41.0; // Inches
    public static final double kLimelightAngle = -91.0; // Degrees 
    public static final double kTargetHeight = 98.25;
  }
}

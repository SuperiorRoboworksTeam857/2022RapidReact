package frc.robot.autonomousCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToTargetCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.List;

public class ThreeBallNearTarmac {

    Trajectory firstTrajectory;
    Trajectory secondTrajectory;
    Trajectory thirdTrajectory;
    Trajectory fourthTrajectory;

    public void generateTrajectories() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig firstConfig =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        TrajectoryConfig secondConfig =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);
        TrajectoryConfig thirdConfig =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        TrajectoryConfig fourthConfig =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);

        firstTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(2, 0, new Rotation2d(0))
                ),
                firstConfig);
        secondTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(2, 0, new Rotation2d(0)),
                new Pose2d(0, 0, new Rotation2d(0))
                ),
                secondConfig);
        thirdTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(0, 0, new Rotation2d(-Math.PI/2)),
                new Pose2d(2, 2*Math.sqrt(3), new Rotation2d(-Math.PI/2))
                ),
                thirdConfig);
        fourthTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(2, 2*Math.sqrt(3), new Rotation2d(-Math.PI/2)),
                new Pose2d(0, 0, new Rotation2d(-Math.PI/2))
                ),
                fourthConfig);
    }


    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(RobotContainer robot) {
    robot.m_robotDrive.resetEncoders();
    robot.m_robotDrive.zeroHeading();
    robot.m_robotIntake.raiseIntake();

    RamseteCommand firstRamseteCommand =
        new RamseteCommand(
            firstTrajectory,
            robot.m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            robot.m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robot.m_robotDrive::tankDriveVolts,
            robot.m_robotDrive);
    RamseteCommand secondRamseteCommand =
        new RamseteCommand(
            secondTrajectory,
            robot.m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            robot.m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robot.m_robotDrive::tankDriveVolts,
            robot.m_robotDrive);
    RamseteCommand thirdRamseteCommand =
        new RamseteCommand(
            thirdTrajectory,
            robot.m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            robot.m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robot.m_robotDrive::tankDriveVolts,
            robot.m_robotDrive);
    RamseteCommand fourthRamseteCommand =
        new RamseteCommand(
            fourthTrajectory,
            robot.m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            robot.m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robot.m_robotDrive::tankDriveVolts,
            robot.m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    robot.m_robotDrive.resetOdometry(firstTrajectory.getInitialPose());

    return new SequentialCommandGroup(
        new InstantCommand(() -> robot.m_robotIntake.lowerIntake(), robot.m_robotIntake),

        // Drive and pick up 2nd ball
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotIntake.runIntake(1), robot.m_robotIntake),
            firstRamseteCommand
        ),
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotIntake.runIntake(1), robot.m_robotIntake),
            secondRamseteCommand
        ),

        // Stop driving and stop intake
        new InstantCommand(() -> robot.m_robotIntake.runIntake(0), robot.m_robotIntake),
        new InstantCommand(() -> robot.m_robotDrive.tankDriveVolts(0, 0)),

        // Target with Limelight
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> robot.m_limelight.enableLimelight(true), robot.m_limelight),
                new ParallelRaceGroup(
                    new TurnToTargetCommand(robot.m_robotDrive, robot.m_limelight, null, 2),
                    new WaitCommand(1)
                ),
                new InstantCommand(() -> robot.m_robotDrive.tankDriveVolts(0, 0)),
                new InstantCommand(() -> robot.m_limelight.enableLimelight(false), robot.m_limelight)
            ),
            new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter)
        ),

        // Shoot 1st ball
        new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter).withInterrupt(robot.m_robotShooter::isShooterAtSpeedUpperHub),
        new InstantCommand(() -> robot.m_robotShooter.raiseKicker(), robot.m_robotShooter ),
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter),
            new WaitCommand(1)
        ),
        new InstantCommand(() -> robot.m_robotShooter.lowerKicker(), robot.m_robotShooter ),

        // Wait for 2nd ball to roll into shooter
        new InstantCommand(() -> robot.m_robotIntake.raiseIntake(), robot.m_robotIntake),
        new WaitCommand(1),
        new WaitCommand(2).withInterrupt(robot.m_robotShooter::isBallPresent),

        // Shoot 2nd ball
        new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter).withInterrupt(robot.m_robotShooter::isShooterAtSpeedUpperHub),
        new InstantCommand(() -> robot.m_robotShooter.raiseKicker(), robot.m_robotShooter ),
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter),
            new WaitCommand(1)
        ),

        // Turn toward 3rd ball
        new TurnToAngleCommand(robot.m_robotDrive, -90, 2),

        // Drive and pick up 3rd ball
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotIntake.runIntake(1), robot.m_robotIntake),
            thirdRamseteCommand
        ),
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotIntake.runIntake(1), robot.m_robotIntake),
            fourthRamseteCommand
        ),

        // Stop driving and stop intake
        new InstantCommand(() -> robot.m_robotIntake.runIntake(0), robot.m_robotIntake),
        new InstantCommand(() -> robot.m_robotDrive.tankDriveVolts(0, 0)),

        // Target with Limelight
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> robot.m_limelight.enableLimelight(true), robot.m_limelight),
                new ParallelRaceGroup(
                    new TurnToTargetCommand(robot.m_robotDrive, robot.m_limelight, null, 2),
                    new WaitCommand(1)
                ),
                new InstantCommand(() -> robot.m_robotDrive.tankDriveVolts(0, 0)),
                new InstantCommand(() -> robot.m_limelight.enableLimelight(false), robot.m_limelight)
            ),
            new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter)
        ),

        // Shoot 3rd ball
        new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter).withInterrupt(robot.m_robotShooter::isShooterAtSpeedUpperHub),
        new InstantCommand(() -> robot.m_robotShooter.raiseKicker(), robot.m_robotShooter ),
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotShooter.runShooterUpperHub(), robot.m_robotShooter),
            new WaitCommand(1)
        ),
        new InstantCommand(() -> robot.m_robotShooter.lowerKicker(), robot.m_robotShooter ),


        new InstantCommand(() -> robot.m_robotShooter.lowerKicker(), robot.m_robotShooter ),
        new InstantCommand(() -> robot.m_robotShooter.stopShooter(), robot.m_robotShooter)

    );


  }
}

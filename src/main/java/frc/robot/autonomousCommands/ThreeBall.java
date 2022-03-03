package frc.robot.autonomousCommands;
import edu.wpi.first.cameraserver.CameraServer;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class ThreeBall {
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand(RobotContainer robot) {
    robot.m_robotDrive.resetEncoders();
    robot.m_robotDrive.zeroHeading();
    robot.m_robotIntake.raiseIntake();

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
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory taxiTrajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.PI/3)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.5, 1, new Rotation2d(0)),
            new Pose2d(3.8, 1, new Rotation2d(0)),
            new Pose2d(5.0, 1.5, new Rotation2d(Math.PI/3))
            ),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            taxiTrajectory,
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
    robot.m_robotDrive.resetOdometry(taxiTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> robot.m_robotDrive.tankDriveVolts(0, 0));

    return new SequentialCommandGroup(
        new InstantCommand(() -> robot.m_robotIntake.lowerIntake(), robot.m_robotIntake),
        new ParallelRaceGroup(
            new RunCommand(() -> robot.m_robotIntake.runIntake(1), robot.m_robotIntake),
            ramseteCommand
        ),
        new RunCommand(() -> robot.m_robotIntake.runIntake(0), robot.m_robotIntake),
        new RunCommand(() -> robot.m_robotDrive.tankDriveVolts(0, 0))
    );
  }
}


// return new SequentialCommandGroup(
//         new ParallelRaceGroup(
//             new RunCommand(() -> m_esophagus.autoRunEsophagus(), m_esophagus),
//             new SequentialCommandGroup(
//                 new InstantCommand(() -> m_intake.lowerIntake(), m_intake),
//                 new ParallelRaceGroup(
//                     firstRamseteCommand,
//                     new RunCommand(() -> m_intake.runIntake(1), m_intake)
//                 ),
//                 new ParallelRaceGroup(
//                     secondRamseteCommand,
//                     new RunCommand(() -> m_intake.runIntake(1), m_intake)
//                 ),
//                 new InstantCommand(() -> m_intake.runIntake(0), m_intake),
//                 new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0), m_robotDrive)
//             )
//         ),
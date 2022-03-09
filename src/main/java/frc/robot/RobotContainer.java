// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ClimberSubsystem m_robotClimber = new ClimberSubsystem();
  public final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  public final ShooterSubsystem m_robotShooter = new ShooterSubsystem();
  public final LimelightSubsystem m_limelight = new LimelightSubsystem();


  Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_driverStick = new Joystick(OIConstants.kDriverStickPort);

  private double topForwardSpeed = 0.7;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    phCompressor.enableAnalog(105, 110);
      
    CameraServer.startAutomaticCapture();
    m_limelight.turnOnDriverCam();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -topForwardSpeed*m_driverStick.getY(), 0.55*m_driverStick.getThrottle()),
            m_robotDrive));

    m_robotIntake.setDefaultCommand(new RunCommand(() -> m_robotIntake.runIntake(0), m_robotIntake));

    m_robotClimber.setDefaultCommand(new RunCommand(() -> m_robotClimber.runArms(m_driverController.getRightY()), m_robotClimber));

    m_robotShooter.setDefaultCommand(new RunCommand(() -> m_robotShooter.runShooter(0), m_robotShooter));

    m_limelight.setDefaultCommand(new RunCommand(() -> m_limelight.enableLimelight(false), m_limelight));

  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverStick, 2).whileHeld(() -> topForwardSpeed = 1)
                                        .whenReleased(() -> topForwardSpeed = 0.7);
    
    new JoystickButton(m_driverStick, 1).whileHeld(() -> m_robotIntake.runIntake(1), m_robotIntake);
    new JoystickButton(m_driverStick, 3).whileHeld(() -> m_robotIntake.runIntake(-1), m_robotIntake);
    new JoystickButton(m_driverStick, 4).whenPressed(() -> m_robotIntake.toggleIntake(), m_robotIntake);
    
    new JoystickButton(m_driverController, 5).whileHeld(() -> m_robotShooter.runShooter(-0.35), m_robotShooter);
    new Trigger(() -> m_driverController.getRightBumper() /*&& m_robotShooter.isShooterAtSpeed()*/)
      .whenActive(() -> m_robotShooter.raiseKicker(), m_robotShooter).whenInactive(() -> m_robotShooter.lowerKicker(), m_robotShooter);
    
      new JoystickButton(m_driverController, 3).whenPressed(() -> m_robotClimber.raiseArms(), m_robotClimber);
    new JoystickButton(m_driverController, 4).whenPressed(() -> m_robotClimber.lowerArms(), m_robotClimber);

    //new JoystickButton(m_driverStick, 3).whenPressed(() -> m_limelight.toggleDriverCam(), m_limelight);
    new JoystickButton(m_driverStick, 5)
      .whenPressed(new SequentialCommandGroup(
            new InstantCommand(() -> m_limelight.enableLimelight(true), m_limelight),
            new TurnToTargetCommand(m_robotDrive, m_limelight, m_driverStick, 50)))
      .whenReleased(() -> m_limelight.enableLimelight(false), m_limelight);
  }

  
}

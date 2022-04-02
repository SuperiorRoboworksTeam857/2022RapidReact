package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;


public class TurnToTargetCommand extends CommandBase {

    private final DriveSubsystem m_robotDrive;
    private final LimelightSubsystem m_limelight;
    private Timer timer = new Timer();
    private double timeout;
    private Joystick m_stick;
    public TurnToTargetCommand(DriveSubsystem subsystem, LimelightSubsystem limelight, Joystick stick, double timeoutS){
        m_robotDrive = subsystem;
        m_limelight = limelight;
        timeout = timeoutS;
        m_stick = stick;
        addRequirements(subsystem, limelight);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute(){
        m_robotDrive.setBrakeMode(true);
        boolean onTarget = false;

        double kP = 0.015;

        // -5 to point left of goal, +5 to point right of goal
        // +2 is pretty well centered
        double tx = m_limelight.getLimelightValue("tx") + 2;
    
        if (m_limelight.getLimelightValue("camMode") == 1) {
            m_limelight.toggleDriverCam();
        }
        double speed = MathUtil.clamp(tx * kP, -0.3, 0.3);
        if (Math.abs(tx) > 2.0 && timer.get() < timeout) {
            m_robotDrive.arcadeDrive(0, speed, false);
        } else {
            onTarget = true;
        }

        if (onTarget) {
            if (m_stick != null) {
                m_robotDrive.arcadeDrive(-0.3 * m_stick.getY(), 0);
            }
        }
    }

    @Override
    public void end(boolean inturrupted){
        m_robotDrive.arcadeDrive(0, 0);
        m_robotDrive.setBrakeMode(false);
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
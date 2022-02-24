package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
  WPI_TalonFX frontMotor = new WPI_TalonFX(ShooterConstants.kFrontMotorPort);
  WPI_TalonFX backMotor = new WPI_TalonFX(ShooterConstants.kBackMotorPort);
    DoubleSolenoid m_kickerSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
     ShooterConstants.kKickerForwardPort,
      ShooterConstants.kKickerReversePort);

    public ShooterSubsystem() {
    
    }
  
    @Override
    public void periodic() {
      
    }
  
    public void runShooter(double speed) {
      frontMotor.set(speed);
      backMotor.set(speed);
    }
  
  
    public void lowerKicker() {
      m_kickerSolenoid.set(Value.kReverse);
    }
  
    public void raiseKicker() {
      m_kickerSolenoid.set(Value.kForward);
    }
}

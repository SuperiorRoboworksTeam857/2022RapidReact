package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
  CANSparkMax leftMotor = new CANSparkMax(ClimberConstants.kLeftMotorPort, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(ClimberConstants.kRightMotorPort, MotorType.kBrushless);
    DoubleSolenoid m_climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
     ClimberConstants.kClimberForwardPort,
      ClimberConstants.kClimberReversePort);

    public ClimberSubsystem() {
    
    }
  
    @Override
    public void periodic() {
      
    }
  
    public void runArms(double speed) {
      leftMotor.set(speed);
      rightMotor.set(speed);
    }
  
  
    public void lowerArms() {
      m_climberSolenoid.set(Value.kReverse);
    }
  
    public void raiseArms() {
      m_climberSolenoid.set(Value.kForward);
    }
}

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      // -250 is top
      //    0 is bottom
      // negative speed goes up, positive speed goes down
      boolean movingUp = (speed < 0);
      boolean movingDown = (speed > 0);

      if ((leftMotor.getEncoder().getPosition() < 0 && movingDown) ||
          (leftMotor.getEncoder().getPosition() > -250 && movingUp)) {
        leftMotor.set(speed);
      }
      else if (movingDown) {
        leftMotor.set(speed*0.1);
      }
      else {
        leftMotor.set(0);
      }
      

      if ((rightMotor.getEncoder().getPosition() < 0 && movingDown) ||
          (rightMotor.getEncoder().getPosition() > -250 && movingUp)) {
            rightMotor.set(speed);
      }
      else if (movingDown) {
        rightMotor.set(speed*0.1);
      }
      else {
        rightMotor.set(0);
      }
      SmartDashboard.putNumber("motor speed", speed);
      SmartDashboard.putNumber("leftArmPosition", leftMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("rightArmPosition", rightMotor.getEncoder().getPosition());
    }
  
  
    public void lowerArms() {
      m_climberSolenoid.set(Value.kReverse);
    }
  
    public void raiseArms() {
      m_climberSolenoid.set(Value.kForward);
    }

    public void resetEncoders() {
      leftMotor.getEncoder().setPosition(0);
      rightMotor.getEncoder().setPosition(0);
    }
}

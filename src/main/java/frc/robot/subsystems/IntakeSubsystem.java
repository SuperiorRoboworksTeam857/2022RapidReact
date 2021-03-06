package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    VictorSP m_intakeMotor = new VictorSP(IntakeConstants.kIntakeMotorPort);
    DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
     IntakeConstants.kIntakeForwardPort,
      IntakeConstants.kIntakeReversePort);

    VictorSP m_agitatorMotor = new VictorSP(IntakeConstants.kIntakeAgitatorPort);

    public IntakeSubsystem() {
    
    }
  
    @Override
    public void periodic() {
      
    }
  
    public void runIntake(double speed) {
      runAgitator(speed);
      if (m_intakeSolenoid.get().equals(Value.kReverse)) {
        m_intakeMotor.set(-speed);
      } else {
        m_intakeMotor.set(0);
      }
    }

    public void runAgitator(double speed) {
      m_agitatorMotor.set(-0.2*speed);
    }
  
    public boolean isIntakeRunning() {
      return m_intakeMotor.get() !=0;
    }
  
    public void toggleIntake(double driveSpeed) {
      if (m_intakeSolenoid.get().equals(Value.kReverse)) {
        m_intakeSolenoid.set(Value.kForward);
      } else if (driveSpeed < 1) {
        m_intakeSolenoid.set(Value.kReverse);
      }
    }
  
    public void raiseIntake() {
      m_intakeSolenoid.set(Value.kForward);
    }
  
    public void lowerIntake() {
      m_intakeSolenoid.set(Value.kReverse);
    }

}

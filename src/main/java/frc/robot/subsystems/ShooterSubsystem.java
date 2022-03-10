package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
    private void runShooter(double speed) {
      frontMotor.set(speed);
      backMotor.set(speed);
      SmartDashboard.putNumber("frontShooterRPM", frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60);
      SmartDashboard.putNumber("backShooterRPM", backMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60);
    }

    public void stopShooter() {
      runShooter(0);
    }

    public void runShooterUpperHub() {
      runShooter(-0.35);
    }

    public void runShooterLowerHub() {
      runShooter(-.23);
    }
  
  
    public void lowerKicker() {
      m_kickerSolenoid.set(Value.kReverse);
    }
  
    public void raiseKicker() {
      m_kickerSolenoid.set(Value.kForward);
    }
    public boolean isShooterAtSpeedUpperHub() {
      return frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60 <= -2000;
    }
    public boolean isShooterAtSpeedLowerHub() {
      return frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60 <= -1000; //<----double check
    }
}

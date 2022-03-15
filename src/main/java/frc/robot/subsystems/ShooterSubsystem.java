package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
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

  public enum Height {NotShooting, ShootingHigh, ShootingLow};
  Height m_height;

  DigitalInput beamBreak = new DigitalInput(4);

  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beam", beamBreak.get());
  }

  private void runShooter(double speed) {
    frontMotor.set(speed);
    backMotor.set(speed);
    SmartDashboard.putNumber("frontShooterRPM", frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60);
    SmartDashboard.putNumber("backShooterRPM", backMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60);
  }

  public void stopShooter() {
    runShooter(0);
    m_height = Height.NotShooting;
  }

  public void runShooterUpperHub() {
    runShooter(-0.35);
    m_height = Height.ShootingHigh;
  }

  public void runShooterLowerHub() {
    runShooter(-.23);
    m_height = Height.ShootingLow;
  }

  public boolean isBallPresent() {
    return beamBreak.get();
  }

  public void lowerKicker() {
    m_kickerSolenoid.set(Value.kReverse);
  }

  public void raiseKicker() {
    boolean overrideBeamBreak = false;
    raiseKicker(overrideBeamBreak);
  }
  public void raiseKicker(boolean overrideBeamBreak) {
    if (isBallPresent() || overrideBeamBreak) {
      m_kickerSolenoid.set(Value.kForward);
    }
  }

  public boolean isShooterAtSpeed() {
    switch (m_height) {
      case ShootingLow:
        return isShooterAtSpeedLowerHub();
      case ShootingHigh:
        return isShooterAtSpeedUpperHub();
      case NotShooting:
        return false;
    }
    return false;
  }

  public boolean isShooterAtSpeedUpperHub() {
    return frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60 <= -2000;
  }
  public boolean isShooterAtSpeedLowerHub() {
    return frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60 <= -1000; //<----double check
  }
}

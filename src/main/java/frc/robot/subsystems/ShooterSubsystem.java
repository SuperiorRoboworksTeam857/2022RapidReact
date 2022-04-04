package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  WPI_TalonFX frontMotor = new WPI_TalonFX(ShooterConstants.kFrontMotorPort);
  WPI_TalonFX backMotor = new WPI_TalonFX(ShooterConstants.kBackMotorPort);
    DoubleSolenoid m_kickerSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
     ShooterConstants.kKickerForwardPort,
      ShooterConstants.kKickerReversePort);

  private final PIDController pidFront = new PIDController(.03, 0, 0);
  private final PIDController pidBack = new PIDController(.03, 0, 0);

  private final SimpleMotorFeedforward m_shooterFrontFeedforward =
    new SimpleMotorFeedforward(0.5754, 0.2116/2); // CAN ID 30
  private final SimpleMotorFeedforward m_shooterBackFeedforward =
    new SimpleMotorFeedforward(0.67431, 0.21763/2); // CAN ID 31

  public enum Height {NotShooting, ShootingHigh, ShootingLow};
  Height m_height;

  DigitalInput beamBreak = new DigitalInput(4);

  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beam", beamBreak.get());
  }

  private void runShooter(double frontSpeed, double backSpeed) {
    double currentFrontSpeedRPS = frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048;
    double currentBackSpeedRPS = backMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048;
    
    double frontVoltage = m_shooterFrontFeedforward.calculate(frontSpeed) +
                          pidFront.calculate(currentFrontSpeedRPS, frontSpeed);
    double backVoltage = m_shooterBackFeedforward.calculate(backSpeed) +
                          pidBack.calculate(currentBackSpeedRPS, backSpeed);
    
    frontMotor.setVoltage(frontVoltage);
    backMotor.setVoltage(backVoltage);

    SmartDashboard.putNumber("frontShooter voltage", frontVoltage);
    SmartDashboard.putNumber("backShooter voltage", backVoltage);

    SmartDashboard.putNumber("frontShooterRPM", frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60);
    SmartDashboard.putNumber("backShooterRPM", backMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60);
  }

  private void runShooter(double speed) {
    runShooter(speed, speed);
  }

  public void stopShooter() {
    runShooter(0);
    m_height = Height.NotShooting;
  }

  public void runShooterUpperHub() {
    //runShooter(-0.35);
    //runShooterVoltage(-4.125, -4.125); //-4.375, 4
    runShooter(-2000/60.0, -2000/60.0);
    m_height = Height.ShootingHigh;
  }

  public void runShooterLowerHub() {
    //runShooter(-.23);
    //runShooter(-2.5); // -2.875
    //runShooterVoltage(-3.25, -1.25);
    runShooter(-1650/60.0, -350/60.0);
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
    return frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60 <= -1950;
  }
  public boolean isShooterAtSpeedLowerHub() {
    return frontMotor.getSensorCollection().getIntegratedSensorVelocity() * 10/2048*60 <= -1600;
  }
}

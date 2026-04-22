package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex leadingShooterMotor = new SparkFlex(13, MotorType.kBrushless);
  private final SparkFlex followingShooterMotorR = new SparkFlex(14, MotorType.kBrushless);
  private final SparkFlex followingShooterMotorL = new SparkFlex(17, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController =
      leadingShooterMotor.getClosedLoopController();

  private final RelativeEncoder leadingShooterEncoder = leadingShooterMotor.getEncoder();

  public ShooterSubsystem() {

    SparkMaxConfig leadingMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followingMotorConfigR = new SparkMaxConfig();
    SparkMaxConfig followingMotorConfigL = new SparkMaxConfig();

    leadingMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .closedLoop
        .p(0.00030)
        .i(0)
        .d(0)
        .maxMotion
        // Set MAXMotion parameters for MAXMotion Velocity control
        .cruiseVelocity(6000)
        .maxAcceleration(50000)
        .allowedProfileError(1);

    followingMotorConfigR
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .follow(leadingShooterMotor, true);

    followingMotorConfigL
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .follow(leadingShooterMotor, false);

    leadingShooterMotor.configure(
        leadingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingShooterMotorR.configure(
        followingMotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingShooterMotorL.configure(
        followingMotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  // Spins the flywheel to a set velocity (RPM)
  public void setTargetVelocity(double targetRPM) {
    flywheelController.setSetpoint(
        targetRPM, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  // Stops the shooter
  public void stopShooter() {
    leadingShooterMotor.stopMotor();
  }

  // Returns arm Rotation
  public double getRPM() {
    return leadingShooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed", getRPM());
    SmartDashboard.putNumber("Shooter Voltage", leadingShooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("3rd motor voltage", followingShooterMotorL.getBusVoltage());
  }
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax leadingShooterMotor = new SparkMax(13, MotorType.kBrushless);
  private final SparkMax followingShooterMotor = new SparkMax(14, MotorType.kBrushless);

  private SparkClosedLoopController flywheelController =
      leadingShooterMotor.getClosedLoopController();

  private final RelativeEncoder leadingShooterEncoder;

  public ShooterSubsystem() {

    SparkMaxConfig leadingMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followingMotorConfig = new SparkMaxConfig();

    leadingMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .closedLoop
        .p(0.00020)
        .i(0)
        .d(0)
        .maxMotion
        // Set MAXMotion parameters for MAXMotion Velocity control
        .cruiseVelocity(5000)
        .maxAcceleration(25000)
        .allowedProfileError(1);

    followingMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .follow(leadingShooterMotor, true);

    leadingShooterMotor.configure(
        leadingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingShooterMotor.configure(
        followingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leadingShooterEncoder = leadingShooterMotor.getEncoder();
  }

  public void setTargetVelocity(double targetRPM) {
    flywheelController.setSetpoint(
        targetRPM, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  // Returns arm Rotation
  public double getRPM() {
    return leadingShooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed", getRPM());
    SmartDashboard.putNumber("Shooter Voltage", leadingShooterMotor.getAppliedOutput());
  }
}

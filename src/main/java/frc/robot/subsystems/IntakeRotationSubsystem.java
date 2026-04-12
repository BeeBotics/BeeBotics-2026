package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRotationSubsystem extends SubsystemBase {
  private final SparkMax leadingRotationMotor = new SparkMax(10, MotorType.kBrushless);
  ;
  private final SparkMax followingRotationMotor = new SparkMax(11, MotorType.kBrushless);
  ;

  private final RelativeEncoder leadingRotationEncoder = leadingRotationMotor.getEncoder();
  ;

  private SparkClosedLoopController rotationController =
      leadingRotationMotor.getClosedLoopController();

  public IntakeRotationSubsystem() {
    SparkMaxConfig leadingMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followingMotorConfig = new SparkMaxConfig();

    leadingMotorConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .closedLoop
        .p(4.25)
        .i(0)
        .d(0)
        .maxMotion
        .cruiseVelocity(5000)
        .maxAcceleration(7000)
        .allowedProfileError(0.1);

    followingMotorConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .follow(leadingRotationMotor, true);

    leadingRotationMotor.configure(
        leadingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingRotationMotor.configure(
        followingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Returns arm Rotation
  public double getRotation() {
    return leadingRotationEncoder.getPosition();
  }

  public void setRotation(double targetRotation) {
    rotationController.setSetpoint(
        targetRotation, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position", getRotation());
  }
}

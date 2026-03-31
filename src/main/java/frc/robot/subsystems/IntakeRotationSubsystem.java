package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRotationSubsystem extends SubsystemBase {
  private final SparkMax leadingRotationMotor;
  private final SparkMax followingRotationMotor;

  private final RelativeEncoder leadingRotationEncoder;
  private final PIDController pid;

  private final double COUNTS_PER_INCH = 42;

  public IntakeRotationSubsystem() {
    leadingRotationMotor = new SparkMax(10, MotorType.kBrushless);
    followingRotationMotor = new SparkMax(11, MotorType.kBrushless);

    SparkMaxConfig leadingMotorConfig = new SparkMaxConfig();
    SparkMaxConfig followingMotorConfig = new SparkMaxConfig();

    leadingMotorConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake).inverted(true);

    followingMotorConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake).inverted(false);

    leadingRotationMotor.configure(
        leadingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followingRotationMotor.configure(
        followingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leadingRotationEncoder = leadingRotationMotor.getEncoder();

    pid = new PIDController(4, 0, 0.4); // Needs tuning
  }

  // Returns arm Rotation
  public double getRotation() {
    return leadingRotationEncoder.getPosition() / COUNTS_PER_INCH;
  }

  public void setRotation(double targetRotation) {
    double output = pid.calculate(getRotation(), targetRotation);
    leadingRotationMotor.set(output);
    followingRotationMotor.set(output);
  }

  public void resetRotation() {
    leadingRotationEncoder.setPosition(0);
  }

  public void setPosition(double targetRotation) {
    pid.setSetpoint(targetRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Good place to update SmartDashboard values if needed
    double pidOutput = pid.calculate(getRotation());
    leadingRotationMotor.set(pidOutput);
    followingRotationMotor.set(pidOutput);
    SmartDashboard.putNumber("Position", getRotation());
  }
}

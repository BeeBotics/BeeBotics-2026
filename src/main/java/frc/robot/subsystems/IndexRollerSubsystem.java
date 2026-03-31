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

public class IndexRollerSubsystem extends SubsystemBase {
  private final SparkFlex indexMotor =
      new SparkFlex(12, MotorType.kBrushless);

  private final RelativeEncoder encoder;

  private SparkClosedLoopController indexController = indexMotor.getClosedLoopController();

  public IndexRollerSubsystem() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .closedLoop
        .p(0.0002)
        .i(0)
        .d(0)
        .maxMotion
        // Set MAXMotion parameters for MAXMotion Velocity control
        .cruiseVelocity(5000)
        .maxAcceleration(25000)
        .allowedProfileError(1);
    ;

    indexMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = indexMotor.getEncoder();
  }

  public double getSpeed() {
    return encoder.getVelocity();
  }

  public void setTargetVelocity(double targetRPM) {
    indexController.setSetpoint(
        targetRPM, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void setVelocity(double targetSpeed) {
    indexMotor.set(targetSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Index Speed", getSpeed());
  }
}

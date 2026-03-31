package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperRollerSubsystem extends SubsystemBase {
  private final SparkMax hopperMotor =
      new SparkMax(16, MotorType.kBrushless);

  private SparkClosedLoopController hopperController = hopperMotor.getClosedLoopController();

  public HopperRollerSubsystem() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .closedLoop
        .p(0.0003)
        .i(0)
        .d(0)
        .maxMotion
        // Set MAXMotion parameters for MAXMotion Velocity control
        .cruiseVelocity(5000)
        .maxAcceleration(20000)
        .allowedProfileError(1);
    ;

    hopperMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(double targetRPM) {
    hopperController.setSetpoint(
        targetRPM, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {}
}

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(15, MotorType.kBrushless);
  ;

  private final RelativeEncoder encoder;

  public IntakeRollerSubsystem() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.smartCurrentLimit(60).idleMode(IdleMode.kCoast).inverted(true);

    intakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = intakeMotor.getEncoder();
  }

  public double getSpeed() {
    return encoder.getVelocity();
  }

  public void setPower(double targetSpeed) {
    intakeMotor.set(targetSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Velocity", getSpeed());
  }
}

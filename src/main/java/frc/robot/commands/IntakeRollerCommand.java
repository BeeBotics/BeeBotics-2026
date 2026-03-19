package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class IntakeRollerCommand extends Command {

  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final double speed;

  public IntakeRollerCommand(IntakeRollerSubsystem m_hopper, double _speed) {
    intakeRollerSubsystem = m_hopper;
    speed = _speed;

    addRequirements(intakeRollerSubsystem);
  }

  @Override
  public void initialize() {
    intakeRollerSubsystem.setPower(speed);
  }

  @Override
  public boolean isFinished() {
    // End the command when the shooter reaches the desired speed
    // return Math.abs(shooterSubsystem.getSpeed() - RPM) < 5500;
    return true;
  }
}

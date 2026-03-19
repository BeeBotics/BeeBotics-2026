package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexRollerSubsystem;

public class IndexRollerCommand extends Command {

  private final IndexRollerSubsystem indexRollerSubsystem;
  private final double power;

  public IndexRollerCommand(IndexRollerSubsystem m_hopper, double _power) {
    indexRollerSubsystem = m_hopper;
    power = _power;

    addRequirements(indexRollerSubsystem);
  }

  @Override
  public void initialize() {
    indexRollerSubsystem.setTargetVelocity(power);
  }

  @Override
  public boolean isFinished() {
    // End the command when the shooter reaches the desired speed
    // return Math.abs(shooterSubsystem.getSpeed() - RPM) < 5500;
    return true;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperRollerSubsystem;

public class HopperRollerCommand extends Command {

  private final HopperRollerSubsystem hopperRollerSubsystem;
  private final double speed;

  public HopperRollerCommand(HopperRollerSubsystem m_hopper, double _speed) {
    hopperRollerSubsystem = m_hopper;
    speed = _speed;

    addRequirements(hopperRollerSubsystem);
  }

  @Override
  public void initialize() {
    hopperRollerSubsystem.setVelocity(speed);
  }

  @Override
  public boolean isFinished() {
    // End the command when the shooter reaches the desired speed
    // return Math.abs(shooterSubsystem.getSpeed() - RPM) < 5500;
    return true;
  }
}

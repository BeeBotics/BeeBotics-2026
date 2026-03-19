package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final double RPM;

  public ShooterCommand(ShooterSubsystem m_shooter, double _RPM) {
    shooterSubsystem = m_shooter;
    RPM = _RPM;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setTargetVelocity(RPM);
  }

  @Override
  public boolean isFinished() {
    // End the command when the shooter reaches the desired speed
    return Math.abs(shooterSubsystem.getSpeed() - RPM) < 4500;
  }
}

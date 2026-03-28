package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommand extends Command {

  private final ShooterSubsystem m_shooter;

  private final DoubleSupplier rpmSupplier;

  // Update constructor
  public ShooterCommand(ShooterSubsystem shooter, DoubleSupplier rpmSupplier) {
    this.m_shooter = shooter;
    this.rpmSupplier = rpmSupplier;
    addRequirements(m_shooter);
  }

  // In the execute() method of the command:
  @Override
  public void execute() {
    m_shooter.setTargetVelocity(rpmSupplier.getAsDouble());
  }
}

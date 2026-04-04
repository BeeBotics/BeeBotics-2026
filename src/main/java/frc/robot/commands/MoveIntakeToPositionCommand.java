package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRotationSubsystem;

public class MoveIntakeToPositionCommand extends Command {

  private final IntakeRotationSubsystem Intake;
  private final double position;

  public MoveIntakeToPositionCommand(IntakeRotationSubsystem m_intake, double _position) {
    Intake = m_intake;
    position = _position;

    addRequirements(Intake);
  }

  @Override
  public void initialize() {
    Intake.setRotation(position);
  }

  @Override
  public boolean isFinished() {
    // End the command when the arm reaches the desired position
    return Math.abs(Intake.getRotation() - position) < 0.04;
    // return true;
  }
}

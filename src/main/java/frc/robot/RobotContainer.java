// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HopperRollerCommand;
import frc.robot.commands.IndexRollerCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.commands.MoveIntakeToPositionCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.HopperRollerSubsystem;
import frc.robot.subsystems.IndexRollerSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeRotationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeRotationSubsystem m_intakeRotation = new IntakeRotationSubsystem();
  private final IntakeRollerSubsystem m_intakeRoller = new IntakeRollerSubsystem();
  private final IndexRollerSubsystem m_indexRoller = new IndexRollerSubsystem();
  private final HopperRollerSubsystem m_hopperRoller = new HopperRollerSubsystem();

  //   private Translation2d Hub = new Translation2d(4.6, 4); // Blue
  private Translation2d Hub = new Translation2d(16.54 - 4.6, 4); // Red

  private final Vision vision;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Declare Named Commands
    NamedCommands.registerCommand(
        "Far Spinup",
        new ShooterCommand(
                m_shooter, () -> 5000) // needs tuned //4775 for high angle // 5000 for low angle
            .alongWith(new IntakeRollerCommand(m_intakeRoller, 0)));
    NamedCommands.registerCommand(
        "Far Spinup2",
        new ShooterCommand(
                m_shooter, () -> 5600) // needs tuned //4775 for high angle // 5000 for low angle
            .alongWith(new IntakeRollerCommand(m_intakeRoller, 0)));
    NamedCommands.registerCommand(
        "Mid Spinup",
        new ShooterCommand(m_shooter, () -> 4400) // needs tuned
            .alongWith(new IntakeRollerCommand(m_intakeRoller, 0)));
    NamedCommands.registerCommand(
        "Shoot",
        new IndexRollerCommand(m_indexRoller, 7000)
            .alongWith(new HopperRollerCommand(m_hopperRoller, 6000)));
    NamedCommands.registerCommand("Intake", new IntakeRollerCommand(m_intakeRoller, 1));
    NamedCommands.registerCommand(
        "Deploy Intake", new MoveIntakeToPositionCommand(m_intakeRotation, 0.07));
    NamedCommands.registerCommand(
        "Stop Shooting",
        new IndexRollerCommand(m_indexRoller, 0)
            .alongWith(new HopperRollerCommand(m_hopperRoller, 0))
            .alongWith(new ShooterCommand(m_shooter, () -> 0)));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
    //   Hub = new Translation2d(16.54 - 4.6, 4); // Red Hub
    // } else {
    //   Hub = new Translation2d(4.6, 4); // Blue Hub
    // }
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -(LimelightHelpers.getTY("") + 7.5) * 0.3,
    //             () -> -controller.getLeftX(),
    //             () -> -(LimelightHelpers.getTX("") + 8) * 0.04));

    // Define your targets (e.g., Center of the Speaker)
    // final Translation2d BLUE_HUB = new Translation2d(4.6, 4);
    // final Translation2d RED_HUB = new Translation2d(16.54 - 4.6, 4);

    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         drive.autoAimDrive(
    //             () -> -controller.getLeftY() * drive.getMaxLinearSpeedMetersPerSec(),
    //             () -> -controller.getLeftX() * drive.getMaxLinearSpeedMetersPerSec(),
    //             DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
    //                 ? RED_HUB
    //                 : RED_HUB));
    controller
        .rightBumper()
        .whileTrue(
            drive.autoAimDrive(
                () -> -controller.getLeftY(), // Forward/Backward (though PID overrides this)
                () -> -controller.getLeftX() // Left/Right (Driver keeps control)
                ));
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               DriveCommands.joystickDriveAtAngle(
    //                 drive,
    //               () -> -controller.getLeftX(),
    //               () -> -controller.getLeftY(),
    //               () -> vision.getTargetX(0));
    //             },
    //             drive));
    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .x()
        .whileTrue(
            // The () -> tells the command to call this function every loop
            new ShooterCommand(m_shooter, () -> drive.getLauncherRPM())
                .alongWith(new IntakeRollerCommand(m_intakeRoller, 0.2)));

    controller
        .y()
        .whileTrue(
            new IndexRollerCommand(m_indexRoller, 8000)
                .alongWith(new HopperRollerCommand(m_hopperRoller, 7000))
            // .alongWith(new MoveIntakeToPositionCommand(m_intakeRotation, 0.0))
            );

    controller
        .a()
        .whileTrue(
            new ShooterCommand(m_shooter, () -> 0)
                .alongWith(new IndexRollerCommand(m_indexRoller, 0))
                .alongWith(new HopperRollerCommand(m_hopperRoller, 0))
                .alongWith(new IntakeRollerCommand(m_intakeRoller, 0)));

    controller
        .b()
        .whileTrue(
            new IntakeRollerCommand(m_intakeRoller, 1)
                .alongWith(new MoveIntakeToPositionCommand(m_intakeRotation, 0.08))
                .alongWith(new IndexRollerCommand(m_indexRoller, 0))
                .alongWith(new HopperRollerCommand(m_hopperRoller, 0))
                .alongWith(new ShooterCommand(m_shooter, () -> 0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

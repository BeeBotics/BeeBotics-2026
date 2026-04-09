// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private LimelightHelpers.PoseEstimate poseEstimate;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private static final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();

  static {
    // shotMap.put(Distance_Meters, Target_RPM);
    shotMap.put(1.5, 3900.0);
    shotMap.put(2.0, 4300.0);
    shotMap.put(2.5, 4600.0);
    shotMap.put(3.0, 5000.0);
    shotMap.put(4.0, 5800.0);
    shotMap.put(5.0, 6600.0);
  }

  private static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();

  static {
    // shotMap.put(Distance_Meters, Target_RPM);
    TOFMap.put(1.5, 0.7);
    TOFMap.put(2.0, 0.9);
    TOFMap.put(2.5, 1.1);
    TOFMap.put(3.0, 1.3);
    TOFMap.put(4.0, 1.5);
    TOFMap.put(4.5, 1.7);
  }

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          Pose2d.kZero,
          VecBuilder.fill(0.1, 0.1, 0.1),
          VecBuilder.fill(0.2, 0.2, 0.1));

  private final Field2d field2d = new Field2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      // Update gyro alert
      LimelightHelpers.SetRobotOrientation(
          "", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

      if (poseEstimate.tagCount > 0) {
        Pose2d visionPose = poseEstimate.pose;
        double distToTag = poseEstimate.avgTagDist;
        double yawVelocity = Math.abs(gyroInputs.yawVelocityRadPerSec);

        boolean rejectUpdate = false;

        if (distToTag > 3.5) {
          rejectUpdate = true;
        }
        if (yawVelocity > Units.degreesToRadians(360)) {
          rejectUpdate = true;
        }
        if (!rejectUpdate) {
          // // Small numbers (0.1) = Trust vision a lot. Large numbers (2.0) = Trust encoders more.
          poseEstimator.addVisionMeasurement(
              visionPose, poseEstimate.timestampSeconds, VecBuilder.fill(0.1, 0.1, 999999999));
        }
      }
      // Adding field map to smart dashboard
      field2d.setRobotPose(getPose());
      SmartDashboard.putData(field2d);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the field-relative velocity (speeds relative to the carpet). */
  @AutoLogOutput(key = "SwerveChassisSpeeds/FieldRelative")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    ChassisSpeeds robotSpeeds = getChassisSpeeds();
    Rotation2d robotRotation = getRotation();

    // Rotate the robot-relative speeds by the robot's heading to get field-relative speeds
    return new ChassisSpeeds(
        robotSpeeds.vxMetersPerSecond * robotRotation.getCos()
            - robotSpeeds.vyMetersPerSecond * robotRotation.getSin(),
        robotSpeeds.vxMetersPerSecond * robotRotation.getSin()
            + robotSpeeds.vyMetersPerSecond * robotRotation.getCos(),
        robotSpeeds.omegaRadiansPerSecond);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Pose/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  public Command autoAim() {
    PIDController thetaController = new PIDController(4.5, 0, 0); // Match your teleop gains
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return run(
        () -> {
          // Get the current translation speeds from PathPlanner's active setpoint
          ChassisSpeeds desiredSpeeds = getChassisSpeeds();

          // Get the target (Blue/Red Hub)
          var alliance = DriverStation.getAlliance();
          Translation2d target =
              (alliance.isPresent() && alliance.get() == Alliance.Red)
                  ? new Translation2d(16.54 - 4.6, 4.0)
                  : new Translation2d(4.6, 4.0);

          // Calculate target rotation
          Translation2d currentTranslation = getPose().getTranslation();
          Translation2d delta = target.minus(currentTranslation);
          Rotation2d angleToTarget = new Rotation2d(delta.getX(), delta.getY());

          Rotation2d targetRotation = angleToTarget.plus(Rotation2d.fromRadians(Math.PI));

          // Calculate PID output for rotation
          double rotationOutput =
              thetaController.calculate(
                  getPose().getRotation().getRadians(), targetRotation.getRadians());
          // Drive
          runVelocity(
              new ChassisSpeeds(
                  desiredSpeeds.vxMetersPerSecond,
                  desiredSpeeds.vyMetersPerSecond,
                  rotationOutput));
        });
  }

  public double getLauncherRPM() {
    Translation2d realTarget = getTargetForZone();

    // Calculate Time of Flight
    double distance = getPose().getTranslation().getDistance(realTarget);
    double timeOfFlight = TOFMap.get(distance);

    // Calculate Virtual Target
    // Subtract the robot's field-relative velocity over the time of flight
    ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
    Translation2d virtualTarget =
        new Translation2d(
            realTarget.getX() - (fieldSpeeds.vxMetersPerSecond * timeOfFlight),
            realTarget.getY() - (fieldSpeeds.vyMetersPerSecond * timeOfFlight));
    double effectiveDistance = getPose().getTranslation().getDistance(virtualTarget);

    // This looks up the distance in the map and interpolates between points
    double targetRPM = shotMap.get(effectiveDistance);

    // Display info on SmartDashboard/AdvantageKit
    Logger.recordOutput("Drive/TargetDistanceMeters", distance);
    Logger.recordOutput("Drive/TargetRPM", targetRPM);
    SmartDashboard.putNumber("Target Distance", distance);
    SmartDashboard.putNumber("Effective Distance", effectiveDistance);
    SmartDashboard.putNumber("Target RPM", targetRPM);

    return targetRPM;
  }

  /** Determines the target translation based on alliance and robot position. */
  private Translation2d getTargetForZone() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

    // Shooting Targets
    Translation2d blueHub = new Translation2d(4.6, 4.0);
    Translation2d redHub = new Translation2d(16.54 - 4.6, 4.0);
    // Passing Targets
    Translation2d bluePassingL = new Translation2d(2.5, 5.5);
    Translation2d bluePassingR = new Translation2d(2.5, 2.5);
    Translation2d redPassingL = new Translation2d(16.54 - 2.5, 5.5);
    Translation2d redPassingR = new Translation2d(16.54 - 2.5, 2.5);

    double robotX = getPose().getX();
    double robotY = getPose().getY();

    if (isRed) {
      // Zone Logic for Red Alliance
      if (robotX > (16.54 - 5.25)) {
        return redHub;
      } else if (robotY > 4) {
        return redPassingL;
      } else {
        return redPassingR;
      }
    } else {
      // Zone Logic for Blue Alliance
      if (robotX < 5.25) {
        return blueHub;
      } else if (robotY > 4) {
        return bluePassingL;
      } else {
        return bluePassingR;
      }
    }
  }

  public Command autoAimDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    // Setup PID for rotation
    PIDController thetaController = new PIDController(4.0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.0));

    return run(
        () -> {
          Translation2d realTarget = getTargetForZone();

          // Calculate Time of Flight
          double distance = getPose().getTranslation().getDistance(realTarget);
          double timeOfFlight = TOFMap.get(distance); // Needs to be tuned

          // Calculate Virtual Target
          // Subtract the robot's field-relative velocity over the time of flight
          ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
          Translation2d virtualTarget =
              new Translation2d(
                  realTarget.getX() - (fieldSpeeds.vxMetersPerSecond * timeOfFlight),
                  realTarget.getY() - (fieldSpeeds.vyMetersPerSecond * timeOfFlight));

          // Aim at the virtual target instead of the real one
          Translation2d currentTranslation = getPose().getTranslation();
          Translation2d delta = virtualTarget.minus(currentTranslation);
          Rotation2d angleToTarget = new Rotation2d(delta.getX(), delta.getY());

          Rotation2d targetRotation = angleToTarget.plus(Rotation2d.fromRadians(Math.PI));

          double rotationOutput =
              thetaController.calculate(
                  getPose().getRotation().getRadians(), targetRotation.getRadians());

          // Drive
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationOutput, getRotation()));

          // Log for AdvantageScope/SmartDashboard
          Logger.recordOutput("Drive/VirtualTarget", virtualTarget);
        });
  }
}

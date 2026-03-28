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
  private Pose2d cameraPose;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private Translation2d Hub;
  double distance = 0;

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

    // LimelightHelpers.SetIMUMode(getName(), 0);
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
      // LimelightHelpers.SetRobotOrientation("", rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);
      poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

      if (poseEstimate.tagCount > 0) {
        Pose2d visionPose = poseEstimate.pose;
        double distToTag = poseEstimate.avgTagDist;

        boolean rejectUpdate = false;

        if (distToTag > 3.0) {
          rejectUpdate = true;
        }
        if (DriverStation.isAutonomous()) {
          rejectUpdate = true;
        }
        if (!rejectUpdate) {
          // // Small numbers (0.1) = Trust vision a lot. Large numbers (2.0) = Trust encoders more.
          poseEstimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
        }
      }
      // // Adding field map to smart dashboard
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

  /** Adds a new timestamped vision measurement to the pose estimator. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    // CRITICAL: You must lock the thread before updating the estimator
    odometryLock.lock();
    try {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } finally {
      odometryLock.unlock();
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  public Command autoAimDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    
    var alliance = DriverStation.getAlliance();
    
    // Define Alliance Hub Locations
    Translation2d blueHub = new Translation2d(4.6, 4.0); 
    Translation2d redHub = new Translation2d(16.54, 4.0);

    // Select the correct target
    Translation2d activeTarget;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        activeTarget = redHub;
    } else {
        activeTarget = blueHub;
    }
    
    // Create a PID controller for rotation
    PIDController thetaController = new PIDController(4.0, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Essential for circular
    // wrap-around

    thetaController.setTolerance(Units.degreesToRadians(1.0)); // 1 degree tolerance

    return run(
        () -> {
          // Calculate the rotation required to face the target
          Translation2d currentTranslation = getPose().getTranslation();
          Translation2d delta = activeTarget.minus(currentTranslation);

          // Use atan2 to get the angle from the robot to the target
          Rotation2d targetRotation = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));

          // Calculate the rotation speed using PID
          double rotationOutput =
              thetaController.calculate(
                  getPose().getRotation().getRadians(), targetRotation.getRadians());

          // Convert supplier values and rotation to ChassisSpeeds
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationOutput, getRotation()));
        });
  }

  public double getLauncherRPM() {
    Pose2d currentPose = getPose();
    Translation2d robotLocation = currentPose.getTranslation();

    var alliance = DriverStation.getAlliance();
    
    // Define Alliance Hub Locations
    Translation2d blueHub = new Translation2d(4.6, 4.0); 
    Translation2d redHub = new Translation2d(16.54, 4.0);

    // Select the correct target
    Translation2d activeTarget;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        activeTarget = redHub;
    } else {
        activeTarget = blueHub;
    }

    double distance = robotLocation.getDistance(activeTarget);

    double targetRPM = (Math.pow((distance), 2) * 100) - (distance * 100) + 4400;

    SmartDashboard.putNumber("Believed Distance", distance);
    SmartDashboard.putNumber("Target RPM", targetRPM);

    return targetRPM;
  }

  // public Command autoAimDrive(
  //     DoubleSupplier xSupplier, DoubleSupplier ySupplier, Translation2d targetTranslation) {
  //   PIDController thetaController = new PIDController(5.0, 0, 0);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   // XY Controllers for movement
  //   PIDController xController = new PIDController(3.0, 0, 0);
  //   PIDController yController = new PIDController(3.0, 0, 0);

  //   return run(
  //       () -> {
  //         Pose2d currentPose = getPose();
  //         Translation2d robotTrans = currentPose.getTranslation();

  //         // Calculate the Angle to the Target
  //         Translation2d delta = targetTranslation.minus(robotTrans);
  //         Rotation2d targetRotation = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));

  //         // Find the direction FROM target TO robot
  //         Translation2d dirFromTarget = robotTrans.minus(targetTranslation);
  //         double distance = dirFromTarget.getNorm();

  //         // Avoid division by zero if we are exactly on the target
  //         Translation2d shotPoint;
  //         if (distance > 0.1) {
  //           shotPoint = targetTranslation.plus(dirFromTarget.times(2.0 / distance));
  //         } else {
  //           shotPoint = targetTranslation.plus(new Translation2d(2.0, 0)); // Default offset
  //         }

  //         // Calculate Velocities to reach that Shot Point
  //         double xVelocity = xController.calculate(robotTrans.getX(), shotPoint.getX());
  //         double yVelocity = yController.calculate(robotTrans.getY(), shotPoint.getY());

  //         // Calculate Rotation
  //         double rotationOutput =
  //             thetaController.calculate(
  //                 currentPose.getRotation().getRadians(), targetRotation.getRadians());

  //         // Adding suppliers allows the driver to fight the auto-align
  //         double finalX = xVelocity + (xSupplier.getAsDouble() * 0.5);
  //         double finalY = yVelocity + (ySupplier.getAsDouble() * 0.5);

  //         runVelocity(
  //             ChassisSpeeds.fromFieldRelativeSpeeds(finalX, finalY, rotationOutput,
  // getRotation()));
  //       });
  // }
}

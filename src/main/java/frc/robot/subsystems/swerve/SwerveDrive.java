package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.gyroIO.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyroIO.GyroInterface;
import frc.robot.subsystems.swerve.moduleIO.ModuleInterface;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThreadInputsAutoLogged;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final GyroInterface gyroIO;
  private final GyroInputsAutoLogged gyroInputs;
  private final OdometryThreadInputsAutoLogged odometryThreadInputs;
  private final SwerveModule[] swerveModules;

  private Rotation2d rawGyroRotation;
  private final SwerveModulePosition[] lastModulePositions;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final OdometryThread odometryThread;

  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro Hardware Fault", Alert.AlertType.kError);

  public SwerveDrive(
      GyroInterface gyroIO,
      ModuleInterface frontLeftModuleIO,
      ModuleInterface frontRightModuleIO,
      ModuleInterface backLeftModuleIO,
      ModuleInterface backRightModuleIO) {
    this.gyroIO = gyroIO;
    this.gyroInputs = new GyroInputsAutoLogged();
    this.rawGyroRotation = new Rotation2d();

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(frontLeftModuleIO, "FrontLeft"),
          new SwerveModule(frontRightModuleIO, "FrontRight"),
          new SwerveModule(backLeftModuleIO, "BackLeft"),
          new SwerveModule(backRightModuleIO, "BackRight")
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(
                DriveConstants.X_POS_TRUST, DriveConstants.Y_POS_TRUST, DriveConstants.ANGLE_TRUST),
            VecBuilder.fill(
                VisionConstants.VISION_X_POS_TRUST,
                VisionConstants.VISION_Y_POS_TRUST,
                VisionConstants.VISION_ANGLE_TRUST));

    this.odometryThread = OdometryThread.createInstance(DeviceCANBus.RIO);
    this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
    this.odometryThread.start();

    gyroDisconnectedAlert.set(false);
  }

  public double getGyroRate() {
    return gyroInputs.yawVelocity;
  }

  /** Updates the pose estimator with the pose calculated from the swerve modules. */
  public void addPoseEstimatorSwerveMeasurement() {
    for (int timeStampIndex = 0;
        timeStampIndex < odometryThreadInputs.measurementTimeStamps.length;
        timeStampIndex++) addPoseEstimatorSwerveMeasurement(timeStampIndex);
  }

  /*
   * Updates the pose estimator with the pose calculated from the april tags. How much it
   * contributes to the pose estimation is set by setPoseEstimatorVisionConfidence.
   *
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the pose from the april tags
   *     was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(
      Pose2d visionMeasurement, double currentTimeStampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  public void setTurnPosition(double position) {
    for (SwerveModule module : swerveModules) {
      module.setTurnPosition(position);
    }
  }

  /**
   * Sets the standard deviations of model states, or how much the april tags contribute to the pose
   * estimation of the robot. Lower numbers equal higher confidence and vice versa.
   *
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(
      double xStandardDeviation, double yStandardDeviation, double thetaStandardDeviation) {
    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

  public void periodic() {
    final double t0 = TimeUtil.getRealTimeSeconds();
    fetchOdometryInputs();
    Logger.recordOutput(
        "SystemPerformance/OdometryFetchingTimeMS", (TimeUtil.getRealTimeSeconds() - t0) * 1000);
    modulesPeriodic();
  }

  public void runCharacterization(double volts) {
    for (SwerveModule module : swerveModules) {
      module.setVoltage(-volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double velocity = 0.0;
    for (SwerveModule module : swerveModules) {
      velocity += module.getCharacterizationVelocity();
    }
    return velocity;
  }

  /** Processes odometry inputs */
  private void fetchOdometryInputs() {
    odometryThread.lockOdometry();
    odometryThread.updateInputs(odometryThreadInputs);
    Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

    for (SwerveModule module : swerveModules) module.updateOdometryInputs();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    gyroDisconnectedAlert.set(!gyroInputs.isConnected);

    odometryThread.unlockOdometry();
  }

  private void modulesPeriodic() {
    for (SwerveModule module : swerveModules) module.periodic();
  }

  /**
   * Tells the robot which way to move
   *
   * @param xSpeed meters per second, max is 5.0
   * @param ySpeed meters per second, max is 5.0
   * @param rotationSpeed radians per second, max is 2
   * @param fieldRelative is the robot field relative
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotationSpeed, getPose().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    setModuleStates(swerveModuleStates);
    Logger.recordOutput("SwerveStates/SwerveModuleStates", swerveModuleStates);
  }

  /** Returns 0 degrees if the robot is on the blue alliance, 180 if on the red alliance. */
  public double getAllianceAngleOffset() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double offset =
        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180.0 : 0.0;
    return offset;
  }

  /**
   * Sets the modules to the specified states.
   *
   * @param desiredStates The desired states for the swerve modules. The order is: frontLeft,
   *     frontRight, backLeft, backRight (should be the same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].runSetPoint(desiredStates[i]);
    }
  }

  private void addPoseEstimatorSwerveMeasurement(int timeStampIndex) {
    final SwerveModulePosition[] modulePositions = getModulesPosition(timeStampIndex),
        moduleDeltas = getModulesDelta(modulePositions);

    if (gyroInputs.isConnected) {
      rawGyroRotation = gyroInputs.odometryYawPositions[timeStampIndex];
    } else {
      Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    poseEstimator.updateWithTime(
        odometryThreadInputs.measurementTimeStamps[timeStampIndex],
        rawGyroRotation,
        modulePositions);
  }

  private SwerveModulePosition[] getModulesPosition(int timeStampIndex) {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[swerveModules.length];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++)
      swerveModulePositions[moduleIndex] =
          swerveModules[moduleIndex].getOdometryPositions()[timeStampIndex];
    return swerveModulePositions;
  }

  private SwerveModulePosition[] getModulesDelta(SwerveModulePosition[] freshModulesPosition) {
    SwerveModulePosition[] deltas = new SwerveModulePosition[swerveModules.length];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      final double deltaDistanceMeters =
          freshModulesPosition[moduleIndex].distanceMeters
              - lastModulePositions[moduleIndex].distanceMeters;
      deltas[moduleIndex] =
          new SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex].angle);
      lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex];
    }
    return deltas;
  }

  // public void stop() {
  //   Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
  //   for (int i = 0; i < swerveHeadings.length; i++) swerveHeadings[i] = new Rotation2d();
  //   DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
  //   HolonomicDriveSubsystem.super.stop();
  // }

  // /**
  //  * Locks the chassis and turns the modules to an X formation to resist movement. The lock will
  // be
  //  * cancelled the next time a nonzero velocity is requested.
  //  */
  // public void lockChassisWithXFormation() {
  //   Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
  //   for (int i = 0; i < swerveHeadings.length; i++)
  //     swerveHeadings[i] = DriveConstants.MODULE_TRANSLATIONS[i].getAngle();
  //   DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
  //   HolonomicDriveSubsystem.super.stop();
  // }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getMeasuredState();
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModuleLatestPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getLatestPosition();
    return states;
  }

  @AutoLogOutput(key = "Odometry/RobotPosition")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRawGyroYaw() {
    return gyroInputs.yawDegreesRotation2d;
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModuleLatestPositions(), pose);
  }
}

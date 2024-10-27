// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveConstants {

  public static final class DriveConstants {
    public static final double X_POS_TRUST = 0.03; // Meters
    public static final double Y_POS_TRUST = 0.03; // Meters
    public static final double ANGLE_TRUST = Units.degreesToRadians(1); // Radians

    // Wheel base and track width are measured by the center of the swerve modules, not the frame of
    // the robot
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(22);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(22);

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
        };

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0 - 9;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 0 - 9;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 0 - 9;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 0 - 9;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 0 - 9;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 0 - 9;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 0 - 9;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 0 - 9;

    public static final int FRONT_LEFT_CANCODER_ID = 0 - 9;
    public static final int FRONT_RIGHT_CANCODER_ID = 0 - 9;
    public static final int REAR_LEFT_CANCODER_ID = 0 - 9;
    public static final int REAR_RIGHT_CANCODER_ID = 0 - 9;

    public static final double FRONT_LEFT_ZERO_ANGLE = 0;
    public static final double FRONT_RIGHT_ZERO_ANGLE = 0;
    public static final double REAR_LEFT_ZERO_ANGLE = 0;
    public static final double REAR_RIGHT_ZERO_ANGLE = 0;

    public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED =
        InvertedValue.Clockwise_Positive;
    public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED =
        InvertedValue.Clockwise_Positive;
    public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED =
        InvertedValue.Clockwise_Positive;
    public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED =
        InvertedValue.Clockwise_Positive;

    public static final InvertedValue FRONT_LEFT_DRIVE_ENCODER_REVERSED =
        InvertedValue.Clockwise_Positive;
    public static final InvertedValue FRONT_RIGHT_DRIVE_ENCODER_REVERSED =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue REAR_LEFT_DRIVE_ENCODER_REVERSED =
        InvertedValue.Clockwise_Positive;
    public static final InvertedValue REAR_RIGHT_DRIVE_ENCODER_REVERSED =
        InvertedValue.CounterClockwise_Positive;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 0 - 9;
    public static final double LOW_ANGULAR_SPEED_RADIANS_PER_SECOND = 0 - 9;

    public static final double MAX_SPEED_METERS_PER_SECOND = 0 - 9;
  }

  public class ModuleConstants {
    public static final double DRIVE_GEAR_RATIO = 0 - 9;
    public static final double TURN_GEAR_RATIO = 0 - 9;
    // Use the wheel diamer characterization periodically to find this
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_TO_METERS_PER_SECOND =
        WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

    public static final double DRIVE_SUPPLY_LIMIT = 45.0;
    public static final double DRIVE_STATOR_LIMIT = 50.0;

    public static final double TURN_P = 0 - 9;
    public static final double TURN_I = 0 - 9;
    public static final double TURN_D = 0 - 9;

    public static final double TURN_S = 0 - 9;
    public static final double TURN_V = 0 - 9;
    public static final double TURN_A = 0 - 9;

    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 0 - 9;
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 0 - 9;

    public static final double DRIVE_P = 0 - 9;
    public static final double DRIVE_I = 0 - 9;
    public static final double DRIVE_D = 0 - 9;
    // Use recalc to find the feedfoward values
    // https://www.reca.lc/drive?appliedVoltageRamp=%7B%22s%22%3A1200%2C%22u%22%3A%22V%2Fs%22%7D&batteryAmpHours=%7B%22s%22%3A18%2C%22u%22%3A%22A%2Ah%22%7D&batteryResistance=%7B%22s%22%3A0.018%2C%22u%22%3A%22Ohm%22%7D&batteryVoltageAtRest=%7B%22s%22%3A12.6%2C%22u%22%3A%22V%22%7D&efficiency=97&filtering=1&gearRatioMax=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&gearRatioMin=%7B%22magnitude%22%3A3%2C%22ratioType%22%3A%22Reduction%22%7D&maxSimulationTime=%7B%22s%22%3A4%2C%22u%22%3A%22s%22%7D&maxSpeedAccelerationThreshold=%7B%22s%22%3A0.15%2C%22u%22%3A%22ft%2Fs2%22%7D&motor=%7B%22quantity%22%3A4%2C%22name%22%3A%22Kraken%20X60%2A%22%7D&motorCurrentLimit=%7B%22s%22%3A60%2C%22u%22%3A%22A%22%7D&numCyclesPerMatch=24&peakBatteryDischarge=20&ratio=%7B%22magnitude%22%3A4.59%2C%22ratioType%22%3A%22Reduction%22%7D&sprintDistance=%7B%22s%22%3A25%2C%22u%22%3A%22ft%22%7D&swerve=1&targetTimeToGoal=%7B%22s%22%3A2%2C%22u%22%3A%22s%22%7D&throttleResponseMax=0.99&throttleResponseMin=0.5&weightAuxilliary=%7B%22s%22%3A24%2C%22u%22%3A%22lbs%22%7D&weightDistributionFrontBack=0.5&weightDistributionLeftRight=0.5&weightInspected=%7B%22s%22%3A125%2C%22u%22%3A%22lbs%22%7D&wheelBaseLength=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D&wheelBaseWidth=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&wheelCOFDynamic=0.9&wheelCOFLateral=1.1&wheelCOFStatic=1.1&wheelDiameter=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D
    public static final double DRIVE_S = 0 - 9;
    public static final double DRIVE_V = 0 - 9;
    public static final double DRIVE_A = 0 - 9;
  }

  public static final class TrajectoryConstants {

    public static final double DRIVE_BASE_DIAMETER =
        Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));
    public static final double MAX_SPEED = 0 - 9;
    public static final double MAX_ACCELERATION = 0 - 9;

    public static final double AUTO_TRANSLATION_P = 0 - 9;
    public static final double AUTO_TRANSLATION_D = 0 - 9;
    public static final double AUTO_THETA_P = 0 - 9;
    public static final double AUTO_THETA_D = 0 - 9;
  }

  /**
   * stores the constants and PID configs for chassis because we want an all-real simulation for the
   * chassis, the numbers are required to be considerably precise
   */
  public class SimulationConstants {
    /**
     * numbers that needs to be changed to fit each robot TODO: change these numbers to match your
     * robot
     */
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 0.95,
        ROBOT_MASS_KG = 40; // with bumpers

    /** TODO: change motor type to match your robot */
    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1),
        STEER_MOTOR = DCMotor.getFalcon500(1);

    public static final double WHEEL_RADIUS_METERS = ModuleConstants.WHEEL_DIAMETER_METERS / 2,
        DRIVE_GEAR_RATIO = ModuleConstants.DRIVE_GEAR_RATIO,
        STEER_GEAR_RATIO = 16.0,
        TIME_ROBOT_STOP_ROTATING_SECONDS = 0.06,
        STEER_FRICTION_VOLTAGE = 0.12,
        DRIVE_FRICTION_VOLTAGE = ModuleConstants.DRIVE_S,
        DRIVE_INERTIA = 0.01,
        STEER_INERTIA = 0.01;

    /* adjust current limit */
    public static final double DRIVE_CURRENT_LIMIT = ModuleConstants.DRIVE_STATOR_LIMIT;
    // public static final double STEER_CURRENT_LIMIT = ModuleConstants.ST;

    /* equations that calculates some constants for the simulator (don't modify) */
    private static final double GRAVITY_CONSTANT = 9.81;
    public static final double DRIVE_BASE_RADIUS = DriveConstants.MODULE_TRANSLATIONS[0].getNorm(),
        /* friction_force = normal_force * coefficient_of_friction */
        MAX_FRICTION_ACCELERATION = GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION,
        MAX_FRICTION_FORCE_PER_MODULE =
            MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG / DriveConstants.MODULE_TRANSLATIONS.length,
        /* force = torque / distance */
        MAX_PROPELLING_FORCE_NEWTONS =
            DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS,
        /* floor_speed = wheel_angular_velocity * wheel_radius */
        CHASSIS_MAX_VELOCITY =
            DRIVE_MOTOR.freeSpeedRadPerSec
                / DRIVE_GEAR_RATIO
                * WHEEL_RADIUS_METERS, // calculate using choreo
        CHASSIS_MAX_ACCELERATION_MPS_SQ =
            Math.min(
                MAX_FRICTION_ACCELERATION, // cannot exceed max friction
                MAX_PROPELLING_FORCE_NEWTONS / ROBOT_MASS_KG),
        CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = CHASSIS_MAX_VELOCITY / DRIVE_BASE_RADIUS,
        CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ =
            CHASSIS_MAX_ACCELERATION_MPS_SQ / DRIVE_BASE_RADIUS,
        CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION =
            CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC / TIME_ROBOT_STOP_ROTATING_SECONDS;

    /* for collision detection in simulation */
    public static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(34.5),
        BUMPER_LENGTH_METERS = Units.inchesToMeters(36),
        /* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
        BUMPER_COEFFICIENT_OF_FRICTION = 0.75,
        /* https://simple.wikipedia.org/wiki/Coefficient_of_restitution */
        BUMPER_COEFFICIENT_OF_RESTITUTION = 0.08;

    /* Gyro Sim */
    public static final double GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ = 100;
    public static final double SKIDDING_AMOUNT_AT_THRESHOLD_RAD = Math.toRadians(1.2);
    /*
     * https://store.ctr-electronics.com/pigeon-2/
     * for a well-installed one with vibration reduction, only 0.4 degree
     * but most teams just install it directly on the rigid chassis frame (including my team :D)
     * so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
     * which is the average velocity during normal swerve-circular-offense
     * */
    public static final double NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD = Math.toRadians(1.2);
    public static final double AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST = Math.toRadians(60);

    public static final int SIMULATION_TICKS_IN_1_PERIOD = 5;
  }

  public static final ModuleConfig[] moduleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
            DriveConstants.FRONT_LEFT_CANCODER_ID,
            DriveConstants.FRONT_LEFT_ZERO_ANGLE,
            DriveConstants.FRONT_LEFT_CANCODER_REVERSED,
            DriveConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            DriveConstants.FRONT_RIGHT_CANCODER_ID,
            DriveConstants.FRONT_RIGHT_ZERO_ANGLE,
            DriveConstants.FRONT_RIGHT_CANCODER_REVERSED,
            DriveConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DriveConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            DriveConstants.REAR_LEFT_TURN_MOTOR_ID,
            DriveConstants.REAR_LEFT_CANCODER_ID,
            DriveConstants.REAR_LEFT_ZERO_ANGLE,
            DriveConstants.REAR_LEFT_CANCODER_REVERSED,
            DriveConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            DriveConstants.REAR_LEFT_DRIVE_ENCODER_REVERSED),
        new ModuleConfig(
            DriveConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            DriveConstants.REAR_RIGHT_TURN_MOTOR_ID,
            DriveConstants.REAR_RIGHT_CANCODER_ID,
            DriveConstants.REAR_RIGHT_ZERO_ANGLE,
            DriveConstants.REAR_RIGHT_CANCODER_REVERSED,
            DriveConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            DriveConstants.REAR_RIGHT_DRIVE_ENCODER_REVERSED)
      };

  public record ModuleConfig(
      int driveMotorChannel,
      int turnMotorChannel,
      int turnEncoderChannel,
      double angleZero,
      SensorDirectionValue encoderReversed,
      InvertedValue turnReversed,
      InvertedValue driveReversed) {}
}

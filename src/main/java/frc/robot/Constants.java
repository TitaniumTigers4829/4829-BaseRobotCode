/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                 */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class HardwareConstants {
    public static final double TIMEOUT_S = 0.03;
    public static final double TIMEOUT_MS = 30;

    public static final String CANIVORE_CAN_BUS_STRING = "canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    public static final double MIN_FALCON_DEADBAND = 0.0001;

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;


  }

  public static final class DriveConstants {

    public static final double X_POS_TRUST = 0-9; //meters
    public static final double Y_POS_TRUST = 0-9; //meters
    public static final double ANGLE_TRUST = 0-9; //radians

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(0-9);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(0-9);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
    );

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;

    public static final int FRONT_LEFT_TURN_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
    public static final int REAR_LEFT_TURN_MOTOR_ID = 7;
    public static final int REAR_RIGHT_TURN_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 11;
    public static final int FRONT_RIGHT_CANCODER_ID = 12;
    public static final int REAR_LEFT_CANCODER_ID = 13;
    public static final int REAR_RIGHT_CANCODER_ID = 14;

    public static final double FRONT_LEFT_ZERO_ANGLE = 0-9;
    public static final double FRONT_RIGHT_ZERO_ANGLE = 0-9;
    public static final double REAR_LEFT_ZERO_ANGLE = 0-9;
    public static final double REAR_RIGHT_ZERO_ANGLE = 0-9;

    //inverts may vary
    public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED = SensorDirectionValue.CounterClockwise_Positive;
    
    public static final InvertedValue FRONT_LEFT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue FRONT_RIGHT_DRIVE_ENCODER_REVERSED = InvertedValue.Clockwise_Positive; 
    public static final InvertedValue REAR_LEFT_DRIVE_ENCODER_REVERSED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue REAR_RIGHT_DRIVE_ENCODER_REVERSED = InvertedValue.Clockwise_Positive;
    
    
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 0-9;

    public static final double MAX_SPEED_METERS_PER_SECOND = 0-9;

  }
  
  public static final class ModuleConstants { 
    public static final double DRIVE_GEAR_RATIO = 0-9;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(0-9);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS =  WHEEL_CIRCUMFERENCE_METERS / (DRIVE_GEAR_RATIO);
    public static final double DRIVE_TO_METERS_PER_SECOND = (WHEEL_CIRCUMFERENCE_METERS) / (DRIVE_GEAR_RATIO);

    public static final double TURN_P = 0-9; 
    public static final double TURN_I = 0-9;
    public static final double TURN_D = 0-9;

    public static final double TURN_S = 0-9;
    public static final double TURN_V = 0-9;
    public static final double TURN_A = 0-9;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 0-9; 
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 0-9;
    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
      );

    public static final double DRIVE_P = 0-9;
    public static final double DRIVE_I = 0-9;
    public static final double DRIVE_D = 0-9;

    public static final double DRIVE_S = 0-9;
    public static final double DRIVE_V = 0-9;
    public static final double DRIVE_A = 0-9;
  }

  public static final class VisionConstants {

    public static final double VISION_X_POS_TRUST = 0-9; //meters
    public static final double VISION_Y_POS_TRUST = 0-9; //meters
    public static final double VISION_ANGLE_TRUST = 0-9; //radians
  
    public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 5;
  
    public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    public static final String BACK_LIMELIGHT_NAME = "limelight-back";

    public static final double[][] APRIL_TAG_POSITIONS = {
      // { x, y, z}
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 1
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 2
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 3
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 4
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 5
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 6
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)}, // 7
      {Units.inchesToMeters(0-9), Units.inchesToMeters(0-9), Units.inchesToMeters(0-9)} // 8
    };

    public static final double[][] CAMERA_CROP_LOOKUP_TABLE = {
      // TODO: All of these are placeholder values
      // {x position in meters, limelight lower y crop}
      {0-9, 0-9},
      {0-9, 0-9},
      {0-9, 0-9},
      {0-9, 0-9},
      {0-9, 0-9}
    };

    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0.0-9, 0-9, 0-9},
      {0-9, 0.0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9}
    };

    public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0.0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9},
      {0-9, 0-9, 0-9, 0-9}
    };
  }

  public static final class ExampleConstants {
    public static final int SOLENOID_FORWARD = 0;
    public static final int SOLENOID_BACKWARD = 1;

    public static final Value SOLENOID_REVERSE_VALUE = Value.kReverse;
    public static final Value SOLENOID_FORWARD_VALUE = Value.kForward;
    public static final Value SOLENOID_OFF = Value.kOff;
  }

  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 0-9;
    public static final double FIELD_WIDTH_METERS = 0-9;
  }

  public static final class JoystickConstants {
    public static final int DRIVER_LEFT_STICK_X = 0-9;
    public static final int DRIVER_LEFT_STICK_Y = 0-9;
    public static final int DRIVER_RIGHT_STICK_X = 0-9;
    public static final int DRIVER_RIGHT_BUMPER_ID = 0-9;

  }
  
}

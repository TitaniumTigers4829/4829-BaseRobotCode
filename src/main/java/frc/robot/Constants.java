package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public class LogPaths {
    public static final String SYSTEM_PERFORMANCE_PATH = "SystemPerformance/";
    public static final String PHYSICS_SIMULATION_PATH = "MaplePhysicsSimulation/";
    public static final String APRIL_TAGS_VISION_PATH = "Vision/AprilTags/";
  }

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * This is where we place constants related to hardware on a robot that aren't specific to any
   * singular subsystem.
   */
  public static final class HardwareConstants {
    public static final double TIMEOUT_S = 0.02;

    public static final double SIGNAL_FREQUENCY = 250;

    public static final String CANIVORE_CAN_BUS_STRING = "canivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    /**
     * For some reason, falcons normally have a deadband threshold of 4%. This is incredibly high!
     * It makes it very hard to do precise movements, so with this constant we set the threshold to
     * the lowest possible value.
     */
    public static final double MIN_FALCON_DEADBAND = 0.001;
  }

  /**
   * This is where constants used to describe the game's field go. This will have the dimensions of
   * the field, but also the coordinates of obstacles, game pieces, or other places of interest.
   */
  public static final class FieldConstants {
    // TODO: Now that I think about it, I'm pretty sure these measurements stay the same every year,
    // so consider setting them in the base code
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(0 - 9);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(0 - 9);
  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final double DEADBAND_VALUE = 0.05;
  }
}

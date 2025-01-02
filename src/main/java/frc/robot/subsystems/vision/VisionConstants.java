package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

public final class VisionConstants {

  /** Enum for using limelight */
  public enum Limelight {
    SHOOTER(VisionConstants.SHOOTER_LIMELIGHT_NUMBER, VisionConstants.SHOOTER_LIMELIGHT_NAME),
    FRONT_LEFT(
        VisionConstants.FRONT_LEFT_LIMELIGHT_NUMBER, VisionConstants.FRONT_LEFT_LIMELIGHT_NAME),
    FRONT_RIGHT(
        VisionConstants.FRONT_RIGHT_LIMELIGHT_NUMBER, VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME);

    private final int id;
    private final String name;

    Limelight(int id, String name) {
      this.id = id;
      this.name = name;
    }

    /**
     * @return The id of the limelight
     */
    public int getId() {
      return id;
    }

    /**
     * @return The name of the limelight
     */
    public String getName() {
      return name;
    }

    /**
     * @param id The id of the limelight
     * @return The Limelight enum value from the id
     */
    public static Limelight fromId(int id) {
      return switch (id) {
        case 0 -> SHOOTER;
        case 1 -> FRONT_LEFT;
        case 2 -> FRONT_RIGHT;
        default -> throw new IllegalArgumentException("Invalid Limelight ID: " + id);
      };
    }
  }

  public static final Transform3d SHOOTER_TRANSFORM =
      new Transform3d(
          new Translation3d(-0.3119324724, 0.0, 0.1865472012), new Rotation3d(0.0, 35, 180.0));
  public static final Transform3d FRONT_LEFT_TRANSFORM =
      new Transform3d(
          new Translation3d(0.2749477356, -0.269958439, 0.2318054546),
          new Rotation3d(0.0, 25, -35));
  public static final Transform3d FRONT_RIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(0.2816630892, 0.2724405524, 0.232156), new Rotation3d(0.0, 25, 35));

  public static final PhotonCamera SHOOTER_CAMERA = new PhotonCamera(Limelight.SHOOTER.getName());
  public static final PhotonCamera FRONT_LEFT_CAMERA =
      new PhotonCamera(Limelight.FRONT_LEFT.getName());
  public static final PhotonCamera FRONT_RIGHT_CAMERA =
      new PhotonCamera(Limelight.FRONT_RIGHT.getName());

  public static final int THREAD_SLEEP_MS = 20;

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double VISION_X_POS_TRUST = 0.5; // meters
  public static final double VISION_Y_POS_TRUST = 0.5; // meters
  public static final double VISION_ANGLE_TRUST = Units.degreesToRadians(50); // radians

  public static final int FRAMES_BEFORE_ADDING_VISION_MEASUREMENT = 2;
  public static final double LL3_FOV_MARGIN_OF_ERROR = 27;
  public static final double LL3G_FOV_MARGIN_OF_ERROR = 38;

  public static final double MEGA_TAG_2_MAX_HEADING_RATE =
      180; // degrees/s // TODO: This can be tested more

  public static final double MEGA_TAG_2_DISTANCE_THRESHOLD = 5; // TODO: Tune

  public static final double MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD = 0.5; // TODO: tune
  public static final double MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD = 45;

  public static final String SHOOTER_LIMELIGHT_NAME = "limelight-shooter";
  public static final int SHOOTER_LIMELIGHT_NUMBER = 0;
  public static final String FRONT_LEFT_LIMELIGHT_NAME = "limelight-left";
  public static final int FRONT_LEFT_LIMELIGHT_NUMBER = 1;
  public static final String FRONT_RIGHT_LIMELIGHT_NAME = "limelight-right";
  public static final int FRONT_RIGHT_LIMELIGHT_NUMBER = 2;

  public static final int MIN_APRIL_TAG_ID = 1;
  public static final int MAX_APRIL_TAG_ID = 16;

  public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
    // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
    {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 2
    {1.5, 0.02, 0.02, Units.degreesToRadians(180000)}, // 5
    {3, 1.2, 1.2, Units.degreesToRadians(180000)}, // 25
    {4.5, 5.5, 5.5, Units.degreesToRadians(180000)}, // 90
    {8, 10.0, 10.0, Units.degreesToRadians(180000)} // 180
  };

  public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
    // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
    {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.5
    {1.5, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.7
    {3, 0.03, 0.03, Units.degreesToRadians(180000)}, // 4
    {4.5, 0.06, 0.06, Units.degreesToRadians(180000)}, // 30
    {8, 0.5, 0.5, Units.degreesToRadians(180000)}, // 90
    {10, 10.0, 10.0, Units.degreesToRadians(180000)} // 90
  };
}

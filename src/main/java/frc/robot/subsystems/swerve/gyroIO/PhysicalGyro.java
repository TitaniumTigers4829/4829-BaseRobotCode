package frc.robot.subsystems.swerve.gyroIO;

import static edu.wpi.first.units.Units.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.subsystems.swerve.gyroIO.GyroInterface.GyroInputs;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import java.util.function.Supplier;

public class PhysicalGyro implements GyroInterface {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 250);

  public PhysicalGyro() {
    OdometryThread.registerInput(getAngle());
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    inputs.yawDegreesRotation2d = getGyroRotation2d();
    inputs.yawVelocity = getRate().in(DegreesPerSecond);
    inputs.rollDegrees = getRoll().in(Degrees);
    inputs.pitchDegrees = getPitch().in(Degrees);
    inputs.yawDegrees = getYaw().in(Degrees);
  }

  public void zeroHeading() {
    gyro.reset();
  }
  
  public Supplier<Angle> getAngle() {
    return () -> Degrees.of(-gyro.getAngle());
  }

  public Angle getYaw() {
    return Degrees.of(-gyro.getAngle());
  }

  public Angle getWrappedYaw() {
    return Degrees.of(-gyro.getYaw());
  }

  public Rotation2d getGyroRotation2d() {
    return gyro.getRotation2d();
  }

  public AngularVelocity getRate() {
    return DegreesPerSecond.of(-gyro.getRate());
  }

  public Angle getPitch() {
    return Degrees.of(gyro.getPitch());
  }

  public Angle getRoll() {
    return Degrees.of(gyro.getRoll());
  }

  public Rotation2d getGyroFieldRelativeRotation2d() {
    if (AllianceFlipper.isBlue()) {
      return getGyroRotation2d();
    }
    return AllianceFlipper.flipRotation(getGyroRotation2d());
  }
}

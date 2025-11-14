package frc.robot.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class PhysicalGyroNavX implements GyroInterface {

  private final AHRS gyro = new AHRS(NavXComType.kUSB2, NavXUpdateRate.k200Hz);

  public PhysicalGyroNavX() {}

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    // These values are negated to make sure CCW is positive for our gyro rotation as is consistent
    // with the rest of pose estimation. We do this
    // because sometimes the gyro can be oriented differently, causing the reading to be negative.
    inputs.yawVelocityDegreesPerSecond = -gyro.getRate();
    inputs.yawDegrees = -gyro.getAngle();
    inputs.accelX = -gyro.getWorldLinearAccelX();
    inputs.accelY = -gyro.getWorldLinearAccelY();
    inputs.rollDegrees = gyro.getRoll();
    inputs.pitchDegrees = gyro.getPitch();
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}

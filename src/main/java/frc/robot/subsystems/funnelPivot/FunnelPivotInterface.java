package frc.robot.subsystems.funnelPivot;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelPivotInterface {
  @AutoLog
  public class FunnelPivotInputs {
    public boolean isConnected = true;
    public double funnelAngle = 0.0;
    public double funnelVoltage = 0.0;
    public double funnelVelocity = 0.0;
    public double funnelTemp = 0.0;
    public double funnelSupplyCurrentAmps = 0.0;
    public double funnelTorqueCurrentAmps = 0.0;
    public double closedLoop = 0.0;
  }

  /** Updates the inputs of the funnel pivot */
  default void updateInputs(FunnelPivotInputs inputs) {}

  /**
   * Sets the funnel speed
   *
   * @param speed the speed to set the funnel to
   */
  default void setFunnelSpeed(double speed) {}

  /**
   * Sets the funnel angle
   *
   * @param angle the angle to set the funnel to
   */
  default void setFunnelAngle(double angle) {}

  /**
   * Sets the funnel voltage
   *
   * @param voltage the voltage to set the funnel to
   */
  default void setFunnelVoltage(double voltage) {}

  /**
   * Checks if the funnel pivot is within an acceptable error
   *
   * @return true if the funnel pivot is within an acceptable error
   */
  default boolean isPivotWithinAcceptapleError() {
    return false;
  }

  /** Returns the angle of the funnel pivot */
  default double getFunnelAngle() {
    return 0.0;
  }

  /** Returns the target angle of the funnel pivot */
  default double getFunnelPivotTarget() {
    return 0.0;
  }

  /**
   * Sets the PID values for the funnel pivot
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   */
  default void setPID(double kP, double kI, double kD) {}

  /** Check for alerts and log them if they exist */
  default void checkAlerts() {}
}

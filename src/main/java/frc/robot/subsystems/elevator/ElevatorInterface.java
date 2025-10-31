package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorInterface {
  @AutoLog
  public static class ElevatorInputs { // For values
    public double leaderMotorPosition = 0.0;
    public double followerMotorPosition = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double followerMotorVoltage = 0.0;
    public double leaderDutyCycle = 0.0;
    public double followerDutyCycle = 0.0;
    public double desiredPosition = 0.0;
    public double leaderStatorCurrent = 0.0;
    public double followerStatorCurrent = 0.0;
    public double leaderVelocity = 0.0;
    public double elevatorError = 0.0;

    public double leaderTemp = 0.0;
    public double followerTemp = 0.0;
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public default double getElevatorPosition() {
    return 0.0;
  }

  public default void setElevatorPosition(double position) {}

  public default void hardStop() {}

  public default void setVolts(double volts) {}

  public default void setPercentOutput(double output) {}

  public default double getVolts() {
    return 0.0;
  }

  public default void enableLimits(boolean forward, boolean reverse) {}

  public default void openLoop(double output) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA, double kG) {}

  public default void resetElevatorPosition(double position) {}

  public default boolean getReverseLimit() {
    return false;
  }

  public default boolean getForwardLimit() {
    return false;
  }
}

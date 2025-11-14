package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeInterface {

  @AutoLog
  public static class CoralIntakeInputs {
    public boolean isMotorConnected = false;
    public boolean isInnerSensorConnected = false;
    public boolean isOuterSensorConnected = false;
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeStatorCurrentAmps = 0.0;
    public double intakePosition = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public boolean hasCoral = false;
    public boolean hasControl = false;
    public double intakeDutyCycle = 0.0;
    public double intakeReference = 0.0;
  }

  default void setIntakeVoltage(double volts) {}

  default boolean hasCoral() {
    return false;
  }

  default boolean hasControl() {
    return false;
  }

  default void updateInputs(CoralIntakeInputs inputs) {}

  default void setIntakeSpeed(double speed) {}

  default double getIntakeSpeed() {
    return 0.0;
  }

  default void setIntakeVelocity(double velocity) {}

  default void setPID(double P) {}
}

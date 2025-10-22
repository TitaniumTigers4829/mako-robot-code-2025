package frc.robot.subsystems.OTBIntake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isConnected = true;

    // intake motor
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;

    // pivot
    public double pivotPosition = 0.0;
    public double pivotVelocity = 0.0;
    public double pivotTemp = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPivotPosition(double position) {}

  default void setIntakeSpeed(double speed) {}

  default double getIntakeSpeed() {
    return 0.0;
  }

  default void setPivotSpeed(double speed) {}

  default double getPivotPosition() {
    return 0.0;
  }
}

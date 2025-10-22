// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.OTBIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /** Set the pivot angle in DEGREES (converted to rotations before sending to IO). */
  public void setPivotAngle(double angleDegrees) {
    // Use whichever of these two lines compiles in your WPILib version:
    // double angleRots = Units.degreesToRotations(angleDegrees); // if available
    double angleRots = Units.radiansToRotations(Units.degreesToRadians(angleDegrees));
    io.setPivotPosition(angleRots);
    Logger.recordOutput("OTBIntake/PivotRotationsCmd", angleRots);
    Logger.recordOutput("OTBIntake/PivotDegreesCmd", angleDegrees);
  }

  /** Set intake roller speed in [-1, 1]. */
  public void setIntakeSpeed(double speed) {
    double clamped = MathUtil.clamp(speed, -1.0, 1.0);
    io.setIntakeSpeed(clamped);
    Logger.recordOutput("OTBIntake/IntakeSpeedCmd", clamped);
  }

  /** Set pivot open-loop speed in [-1, 1] (useful for jog). */
  public void setPivotSpeed(double speed) {
    double clamped = MathUtil.clamp(speed, -1.0, 1.0);
    io.setPivotSpeed(clamped);
    Logger.recordOutput("OTBIntake/PivotSpeedCmd", clamped);
  }

  /** Current pivot position (rotations, unless your IO says otherwise). */
  public double getPivotPosition() {
    return io.getPivotPosition();
  }

  /** Optional helpers. */
  public void stopIntake() {
    io.setIntakeSpeed(0.0);
  }

  public void stopPivot() {
    io.setPivotSpeed(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OTBIntake", inputs);
    // If your IO inputs include angles, log converted units too:
    // Logger.recordOutput("OTBIntake/PivotRotationsMeas", inputs.pivotPositionRots);
    // Logger.recordOutput("OTBIntake/PivotDegreesMeas",
    //     Units.radiansToDegrees(Units.rotationsToRadians(inputs.pivotPositionRots)));
  }
}

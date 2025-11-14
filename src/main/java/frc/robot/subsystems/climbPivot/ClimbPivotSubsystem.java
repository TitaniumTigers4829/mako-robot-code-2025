// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** This */
public class ClimbPivotSubsystem extends SubsystemBase {
  /** Creates a new ClimbPivot. */
  ClimbPivotInterface climbPivotInterface;

  ClimbPivotInputsAutoLogged inputsAutoLogged = new ClimbPivotInputsAutoLogged();

  public ClimbPivotSubsystem(ClimbPivotInterface climbPivotInterface) {
    this.climbPivotInterface = climbPivotInterface;
  }

  /** angle is in rotations, idk where the angles are or what 0 is */
  public void setClimbPivotPosition(double angle) {
    climbPivotInterface.setClimbPivotPosition(angle);
  }

  /** in rotations, i think that's a problem y'all */
  public double getClimbPivotPosition() {
    return climbPivotInterface.getClimbPivotPosition();
  }

  public double getVolts() {
    return climbPivotInterface.getVolts();
  }

  /**
   * in rotations, endAngle is something i guess i need help
   *
   * @param endAngle this is the angle where the pivot ends
   */
  public void stop(double endAngle) {
    climbPivotInterface.setClimbPivotPosition(endAngle);
  }

  /**
   * Sets the pivot motor speed manually from -1 to 1
   *
   * @param speed the percent speed from -1 to 1
   */
  public void manualPivot(double speed) {
    climbPivotInterface.manualPivot(speed);
  }

  @Override
  public void periodic() {
    climbPivotInterface.updateInputs(inputsAutoLogged);
    Logger.processInputs("Climb Pivot/", inputsAutoLogged);
  }

  /**
   * actually pivots the thing to a set angle
   *
   * @param setAngle the command that sets it to the angle
   * @return
   */
  public Command setAngle() {
    return new StartEndCommand(
            () -> setClimbPivotPosition(PivotConstants.TARGET_POSITION),
            () -> stop(PivotConstants.TARGET_POSITION),
            this)
        .until(() -> isPivotCloseEnough());
  }

  /**
   * @param position i dont actually know what this is
   * @return
   */
  public Command manualPivotClimb(DoubleSupplier position) {
    return new RunCommand(
        () -> climbPivotInterface.manualPivot(MathUtil.applyDeadband(position.getAsDouble(), .1)),
        this);
  }

  /**
   * Private helper method to check if the pivot is close enough to the target position
   *
   * @return
   */
  private boolean isPivotCloseEnough() {
    return Math.abs(getClimbPivotPosition() - PivotConstants.TARGET_POSITION)
        < PivotConstants.PIVOT_ACCEPTABLE_ERROR_DEGREES;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.example;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class ExampleSubsystem extends SubsystemBase {
  private DoubleSupplier DOUBLELOL;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  // this is the interface for the subsystem
  // code for motors and such would go here

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void exampleSubsystemFunction() {}

  private void setExamplePivotAngle(double angle) {
    // normal subsystem stuff
  }

  private double getExamplePivotAngle() {
    return 0.0;
  }

  private boolean isAngleSet() {
    return false;
  }

  private boolean isGamepieceinRobot() {
    return false;
  }

  // Public Command Factory
  // create these for everything you would need to access from a command
  // so you don't input values or acces the internals of a subsystem in the commands themselves
  public Command exampleFunctionalCommand() {
    // the command is written here instead of in it's own seperate file.
    // this saves space and makes the code more readable

    return new FunctionalCommand(
        // does on init
        () -> this.setExamplePivotAngle(0),
        // does on execute
        () -> this.setExamplePivotAngle(1),
        // does when command ends
        interrupted -> this.setExamplePivotAngle(0),
        // ends the command when this is true
        () -> getExamplePivotAngle() >= 0,
        // requirements for the command
        this);
  }

  // Shared internal implementation
  private Command exampleStartEndCommand() {
    // the command is written here instead of in it's own seperate file.
    // this saves space and makes the code more readable

    return new StartEndCommand(
        // Sets the pivot to 1 while the command is active
        () -> this.setExamplePivotAngle(1),
        // Sets the pivot to 0 when the command ends
        () -> this.setExamplePivotAngle(0),
        // requirements for the command
        this);
  }

  // Shared internal implementation
  private Command exampleRunOnceCommand(double angle) {
    return this.runOnce(() -> setExamplePivotAngle(angle));
  }

  // Public Command Factory
  public Command setPivotToScore() {
    return exampleRunOnceCommand(ExampleConstants.SHOOTER_PIVOT_ANGLE);
  }

  // Public Command Factory
  public Command setPivotToAngle(DoubleSupplier angle) {
    // if you need to pass a value in do it this way
    return exampleRunOnceCommand(angle.getAsDouble());
  }

  // this is a trigger
  // if you need to give information from the subsystem do it like this
  // must be a boolean
  public final Trigger isAngleInRightSpot = new Trigger(() -> isAngleSet());

  // another trigger example
  public final Trigger hasGamepiece = new Trigger(() -> isGamepieceinRobot());
}

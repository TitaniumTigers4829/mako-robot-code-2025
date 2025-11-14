// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeSubsystem extends SubsystemBase {
  private CoralIntakeInterface coralIntakeInterface;
  private CoralIntakeInputsAutoLogged coralIntakeInputs = new CoralIntakeInputsAutoLogged();

  // States for the intake process
  public enum IntakeState {
    IDLE, // Chilling, not doing anything
    WAITING, // Waiting for coral to show up
    INGESTING, // Pulling the coral in
    REVERSING, // Backing up to position it
    STOPPED // All done, motor off
  }

  private IntakeState currentState = IntakeState.IDLE;
  private boolean usedToHaveCoral = false;
  private boolean usedToHaveControl = false;

  public CoralIntakeSubsystem(CoralIntakeInterface coralIntakeInterface) {
    this.coralIntakeInterface = coralIntakeInterface;
  }

  /**
   * Sets the intake to a desired speed
   *
   * @param speed the speed to set.
   */
  public void setIntakeSpeed(double speed) {
    coralIntakeInterface.setIntakeSpeed(speed);
  }

  /**
   * Checks if the coral intake has a coral or not
   *
   * @return true if the intake contains a coral game piece.
   */
  public boolean hasCoral() {
    return coralIntakeInputs.hasCoral;
  }

  public void intakeCoral() {
    if (isIntakeIdle()) {
      currentState = IntakeState.WAITING;
      usedToHaveCoral = hasCoral(); // Set the starting sensor state
      usedToHaveControl = hasControl();
    }
  }

  public boolean hasControl() {
    return coralIntakeInputs.hasControl;
  }

  // Tells you if the intake is done
  public boolean isIntakeComplete() {
    return currentState == IntakeState.STOPPED;
  }

  public boolean isIntakeIdle() {
    return currentState == IntakeState.IDLE;
  }

  @Override
  public void periodic() {
    coralIntakeInterface.updateInputs(coralIntakeInputs);
    Logger.processInputs("CoralIntakeSubsystem/", coralIntakeInputs);

    boolean currentlyHasCoral = hasCoral();
    boolean currentlyHasControl = hasControl();

    // Run the state machine
    switch (currentState) {
      case WAITING:
        coralIntakeInterface.setIntakeVelocity(CoralIntakeConstants.WAITING_INTAKE_SPEED);
        if ((currentlyHasControl && !usedToHaveControl)) {
          // Coral just got detected: start pushing it out
          coralIntakeInterface.setIntakeVelocity(CoralIntakeConstants.INGEST_SPEED);
          currentState = IntakeState.INGESTING;
        }
        break;

      case INGESTING:
        if (!currentlyHasControl && currentlyHasCoral) {
          // Coral’s gone from the sensor: reverse to position it
          coralIntakeInterface.setIntakeVelocity(CoralIntakeConstants.REVERSE_INTAKE_SPEED);
          currentState = IntakeState.REVERSING;
        }
        break;

      case REVERSING:
        if (currentlyHasCoral && currentlyHasControl) {
          // Coral’s back in place: stop the motor
          coralIntakeInterface.setIntakeVelocity(CoralIntakeConstants.NEUTRAL_INTAKE_SPEED);
          currentState = IntakeState.STOPPED;
        }
        break;

      case STOPPED:
        coralIntakeInterface.setIntakeVelocity(CoralIntakeConstants.NEUTRAL_INTAKE_SPEED);
        break;

      case IDLE:
        coralIntakeInterface.setIntakeVelocity(CoralIntakeConstants.NEUTRAL_INTAKE_SPEED);
        // Waiting for the signal to start
        break;
    }

    // Remember the sensor state for next time
    usedToHaveCoral = currentlyHasCoral;
    usedToHaveControl = currentlyHasControl;

    SmartDashboard.putBoolean("hasCoral", coralIntakeInterface.hasCoral());
    SmartDashboard.putBoolean("hasControl", coralIntakeInterface.hasControl());
  }

  public void setIntakeState(IntakeState state) {
    currentState = state;
  }

  public double getIntakeVelocity() {
    return coralIntakeInputs.intakeVelocity;
  }

  public void setIntakeVoltage(double volts) {
    coralIntakeInterface.setIntakeVoltage(volts);
  }

  public void setIntakeVelocity(double velocity) {
    coralIntakeInterface.setIntakeVelocity(velocity);
  }

  // public Command intakeCoral() {
  //   if (!this.hasCoral()) {
  //     return new StartEndCommand(
  //         // sets speed while command is active
  //         () -> {
  //           this.setIntakeSpeed(CoralIntakeConstants.INGEST_SPEED);
  //           ledSubsystem.setProcess(LEDProcess.ORANGE);
  //         },
  //         // sets speed when command ends
  //         () -> {
  //           this.setIntakeSpeed(0);
  //           ledSubsystem.setProcess(LEDProcess.ALLIANCE_COLOR);
  //         },
  //         // requirements for command
  //         this);
  //   } else {
  //     return new StartEndCommand(
  //         // sets speed while command is active
  //         () -> {
  //           this.setIntakeSpeed(0.0);
  //           ledSubsystem.setProcess(LEDProcess.GREEN);
  //         },
  //         // sets speed when command ends
  //         () -> {
  //           this.setIntakeSpeed(0);
  //           ledSubsystem.setProcess(LEDProcess.ALLIANCE_COLOR);
  //         },
  //         // requirements for command
  //         this);
  //   }
  // }

  public Command ejectCoral() {
    return new StartEndCommand(
        // sets speed while command is active
        () -> this.setIntakeVelocity(CoralIntakeConstants.EJECT_SPEED),
        // sets speed when command ends
        () -> this.setIntakeVelocity(0),
        // requirements for command
        this);
  }
}

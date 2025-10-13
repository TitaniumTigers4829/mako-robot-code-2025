// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorInterface elevatorInterface;
  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  private static final LoggedTunableNumber elevatorS =
      new LoggedTunableNumber("Elevator/ElevatorS");
  private static final LoggedTunableNumber elevatorV =
      new LoggedTunableNumber("Elevator/ElevatorV");
  private static final LoggedTunableNumber elevatorA =
      new LoggedTunableNumber("Elevator/ElevatorA");
  private static final LoggedTunableNumber elevatorG =
      new LoggedTunableNumber("Elevator/ElevatorG");
  private static final LoggedTunableNumber elevatorP =
      new LoggedTunableNumber("Elevator/ElevatorP");
  private static final LoggedTunableNumber elevatorI =
      new LoggedTunableNumber("Elevator/ElevatorI");
  private static final LoggedTunableNumber elevatorD =
      new LoggedTunableNumber("Elevator/ElevatorD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        elevatorS.initDefault(ElevatorConstants.ELEVATOR_S);
        elevatorV.initDefault(ElevatorConstants.ELEVATOR_V);
        elevatorA.initDefault(ElevatorConstants.ELEVATOR_A);
        elevatorG.initDefault(ElevatorConstants.ELEVATOR_G);
        elevatorP.initDefault(ElevatorConstants.ELEVATOR_P);
        elevatorI.initDefault(ElevatorConstants.ELEVATOR_I);
        elevatorD.initDefault(ElevatorConstants.ELEVATOR_D);
      }
      default -> {
        elevatorS.initDefault(0.014);
        elevatorV.initDefault(0.134);
        elevatorA.initDefault(0.1);
        elevatorG.initDefault(0);
        elevatorP.initDefault(10.0);
        elevatorI.initDefault(0);
        elevatorD.initDefault(0);
      }
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorInterface elevatorInterface) {
    this.elevatorInterface = elevatorInterface;
    enableLimits(true, true);
  }

  public double getElevatorPosition() {
    return elevatorInterface.getElevatorPosition();
  }

  public double getVolts() {
    return elevatorInterface.getVolts();
  }

  public void setElevatorPosition(double position) {
    elevatorInterface.setElevatorPosition(position);
  }

  public double getVelocity() {
    return inputs.leaderVelocity;
  }

  public void setVolts(double volts) {
    elevatorInterface.setVolts(volts);
  }

  public void openLoop(double output) {
    elevatorInterface.openLoop(output);
  }

  public void setPercentOutput(double output) {
    elevatorInterface.setPercentOutput(output);
  }

  public boolean isAtSetpoint(double position) {
    return Math.abs(position - inputs.leaderMotorPosition)
        < ElevatorConstants.ELEVATOR_ERROR_TOLERANCE;
  }

  public void resetPosition(double position) {
    elevatorInterface.resetElevatorPosition(position);
  }

  public void enableLimits(boolean forward, boolean reverse) {
    elevatorInterface.enableLimits(forward, reverse);
  }

  public void toggleLimits() {
    elevatorInterface.enableLimits(
        !elevatorInterface.getForwardLimit(), !elevatorInterface.getReverseLimit());
  }

  @Override
  public void periodic() {
    elevatorInterface.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);

    // Update tunable numbers
    if (elevatorS.hasChanged(hashCode())
        || elevatorV.hasChanged(hashCode())
        || elevatorA.hasChanged(hashCode())
        || elevatorG.hasChanged(hashCode())) {
      elevatorInterface.setFF(elevatorS.get(), elevatorV.get(), elevatorA.get(), elevatorG.get());
    }
    if (elevatorP.hasChanged(hashCode())
        || elevatorI.hasChanged(hashCode())
        || elevatorD.hasChanged(hashCode())) {
      elevatorInterface.setPID(elevatorP.get(), elevatorI.get(), elevatorD.get());
    }
  }

  public Command manualElevator(DoubleSupplier joystickY) {
    return new RunCommand(
        // does this while command is active
        () -> this.openLoop(MathUtil.applyDeadband(joystickY.getAsDouble(), .1)),
        // requirements for command
        this);
  }

  public Command manualElevator(double output) {
    return new RunCommand(
        // does this while command is active
        () -> this.openLoop(output),
        // requirements for command
        this);
  }

  public Command setElevationPosition(double position) {
    return new FunctionalCommand(
        // initialization function
        () -> {},
        // execution function
        () -> this.setElevatorPosition(position),
        // end function
        interrupted -> this.setElevatorPosition(position),
        // isFinished function
        () -> isAtSetpoint(position),
        this);
  }
}

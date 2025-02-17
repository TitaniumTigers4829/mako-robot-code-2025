// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeSubsystem extends SubsystemBase {
  private CoralIntakeInterface intakeInterface;
  private CoralIntakeInputsAutoLogged inputs = new CoralIntakeInputsAutoLogged();

  public CoralIntakeSubsystem(CoralIntakeInterface intakeInterface) {
    this.intakeInterface = intakeInterface;
  }

  public void setIntakeSpeed(double speed) {
    intakeInterface.setIntakeSpeed(speed);
  }

  @Override
  public void periodic() {
    intakeInterface.updateInputs(inputs);
    Logger.processInputs("IntakeSubsystem/", inputs);
  }

  public Command Intake(){
    return new StartEndCommand(
      //sets speed while command is active
      () -> this.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED), 
      //sets speed when command ends
      () -> this.setIntakeSpeed(0), 
      //requirements for command
      this);
  }

  public Command Eject(){
    return new StartEndCommand(
      //sets speed while command is active
      () -> this.setIntakeSpeed(CoralIntakeConstants.EJECT_SPEED), 
      //sets speed when command ends
      () -> this.setIntakeSpeed(0), 
      //requirements for command
      this);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimulatedElevator implements ElevatorInterface {
  private final ElevatorSim m_elevatorSim;
  private final PIDController m_pidController;
  private final ElevatorFeedforward m_feedforward;
  private double currentVolts;

  public SimulatedElevator() {
    m_elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.ELEVATOR_GEAR_RATIO,
            ElevatorConstants.ELEVATOR_CARRIAGE_MASS,
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            ElevatorConstants.SIMULATE_GRAVITY,
            ElevatorConstants.MIN_HEIGHT);
    m_pidController =
        new PIDController(
            ElevatorConstants.ELEVATOR_P,
            ElevatorConstants.ELEVATOR_I,
            ElevatorConstants.ELEVATOR_D);
    m_feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.ELEVATOR_S,
            ElevatorConstants.ELEVATOR_G,
            ElevatorConstants.ELEVATOR_V,
            ElevatorConstants.ELEVATOR_A);
    currentVolts = 0.0;
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    m_elevatorSim.update(0.02);
    inputs.leaderMotorPosition = m_elevatorSim.getPositionMeters();
    inputs.followerMotorPosition = m_elevatorSim.getPositionMeters();
    inputs.leaderMotorVoltage = currentVolts;
    inputs.followerMotorVoltage = currentVolts;
  }

  @Override
  public double getElevatorPosition() {
    return m_elevatorSim.getPositionMeters();
  }

  @Override
  public void setElevatorPosition(double position) {
    m_pidController.setSetpoint(position);
    double output = m_pidController.calculate(getElevatorPosition(), position);
    double feedforward = m_feedforward.calculate(m_pidController.getSetpoint());
    setVolts(output + feedforward);
  }

  @Override
  public void setVolts(double volts) {
    currentVolts = volts;
    m_elevatorSim.setInputVoltage(currentVolts);
  }

  @Override
  public double getVolts() {
    return currentVolts;
  }
}

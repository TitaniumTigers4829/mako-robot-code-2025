// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalElevator implements ElevatorInterface {
  private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
  private final TalonFX followerMotor = new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Double> followerDutyCycle;
  private final StatusSignal<Double> leaderDutyCycle;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Double> elevatorReference;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Temperature> followerTemp;
  private final double elevatorError;

  private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  /** Creates a new PhysicalElevator. */
  public PhysicalElevator() {
    // Create elevator config

    // Static gravity compensation
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Limits
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.REVERSE_LIMIT;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.LIMIT;

    // Motor neutral output is Brake mode
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set current limits
    elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable =
        ElevatorConstants.STATOR_CURRENT_LIMIT_ENABLE;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ElevatorConstants.SUPPLY_CURRENT_LIMIT_ENABLE;

    // Set inverted
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.MOTION_MAGIC_MAX_ACCELERATION;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;

    // Set gearing
    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEAR_RATIO;

    leaderMotor.getConfigurator().apply(elevatorConfig);

    // Use Follower control
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerMotor.getConfigurator().apply(elevatorConfig);

    leaderPosition = leaderMotor.getPosition();
    leaderAppliedVoltage = leaderMotor.getMotorVoltage();
    followerPosition = followerMotor.getPosition();
    followerAppliedVoltage = followerMotor.getMotorVoltage();
    followerDutyCycle = followerMotor.getDutyCycle();
    leaderDutyCycle = leaderMotor.getDutyCycle();
    leaderStatorCurrent = leaderMotor.getStatorCurrent();
    followerStatorCurrent = followerMotor.getStatorCurrent();
    elevatorReference = followerMotor.getClosedLoopReference();
    leaderVelocity = leaderMotor.getVelocity();
    elevatorError =
        followerMotor.getClosedLoopReference().getValueAsDouble()
            - followerMotor.getPosition().getValueAsDouble();
    leaderTemp = leaderMotor.getDeviceTemp();
    followerTemp = followerMotor.getDeviceTemp();

    leaderMotor.setPosition(0.0);
    followerMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.RIO_SIGNAL_FREQUENCY,
        leaderPosition,
        leaderAppliedVoltage,
        followerPosition,
        followerAppliedVoltage,
        leaderDutyCycle,
        followerDutyCycle,
        leaderStatorCurrent,
        followerStatorCurrent,
        elevatorReference,
        leaderVelocity,
        followerTemp,
        leaderTemp);
    followerMotor.optimizeBusUtilization();
    leaderMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        followerPosition,
        leaderAppliedVoltage,
        followerAppliedVoltage,
        followerDutyCycle,
        leaderDutyCycle,
        leaderStatorCurrent,
        followerStatorCurrent,
        elevatorReference,
        leaderVelocity,
        leaderTemp,
        followerTemp);
    inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
    inputs.leaderMotorVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderDutyCycle = leaderDutyCycle.getValueAsDouble();
    inputs.followerMotorPosition = followerPosition.getValueAsDouble();
    inputs.followerMotorVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerDutyCycle = followerDutyCycle.getValueAsDouble();
    inputs.desiredPosition = elevatorReference.getValueAsDouble();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValueAsDouble();
    inputs.followerStatorCurrent = followerStatorCurrent.getValueAsDouble();
    inputs.leaderVelocity = leaderVelocity.getValueAsDouble();
    inputs.elevatorError = elevatorError;
    inputs.leaderTemp = leaderTemp.getValueAsDouble();
    inputs.followerTemp = followerTemp.getValueAsDouble();
  }

  @Override
  public double getElevatorPosition() {
    leaderPosition.refresh();
    followerPosition.refresh();
    return leaderPosition.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double position) {
    leaderMotor.setControl(mmPositionRequest.withPosition(position));
    followerMotor.setControl(mmPositionRequest.withPosition(-position));
  }

  @Override
  public void hardStop() {
    if (leaderStatorCurrent.getValueAsDouble() > ElevatorConstants.STATOR_CURRENT_THRESHOLD
        && followerStatorCurrent.getValueAsDouble() > ElevatorConstants.STATOR_CURRENT_THRESHOLD) {
      leaderMotor.setControl(mmPositionRequest.withPosition(-0.001));
      followerMotor.setControl(mmPositionRequest.withPosition(0.001));
    }
  }

  @Override
  public void setVolts(double volts) {
    leaderMotor.setVoltage(-volts);
    followerMotor.setVoltage(volts);
  }

  @Override
  public double getVolts() {
    return leaderAppliedVoltage.getValueAsDouble();
  }

  @Override
  public void openLoop(double output) {
    leaderMotor.setControl(dutyCyleOut.withOutput(output));
    followerMotor.setControl(dutyCyleOut.withOutput(-output));
  }

  @Override
  public void resetElevatorPosition(double position) {
    leaderMotor.setPosition(position);
    followerMotor.setPosition(position);
  }

  @Override
  public void enableLimits(boolean forward, boolean reverse) {
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    leaderMotor.getConfigurator().apply(elevatorConfig);
  }

  @Override
  public boolean getForwardLimit() {
    return elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
  }

  @Override
  public boolean getReverseLimit() {
    return elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    elevatorConfig.Slot0.kP = kP;
    elevatorConfig.Slot0.kI = kI;
    elevatorConfig.Slot0.kD = kD;
    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    elevatorConfig.Slot0.kS = kS;
    elevatorConfig.Slot0.kV = kV;
    elevatorConfig.Slot0.kA = kA;
    elevatorConfig.Slot0.kG = kG;
    leaderMotor.getConfigurator().apply(elevatorConfig);
    followerMotor.getConfigurator().apply(elevatorConfig);
  }
}

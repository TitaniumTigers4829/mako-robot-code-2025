// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class PhysicalClimbPivot implements ClimbPivotInterface {
  public TalonFXConfiguration config;
  private TalonFX climbMotor = new TalonFX(PivotConstants.CLIMB_PIVOT_MOTOR_ID);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

  private final StatusSignal<Voltage> climbMotorAppliedVoltage;
  private final StatusSignal<Angle> climbMotorAngle;

  public PhysicalClimbPivot() {
    config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = PivotConstants.CLIMB_PIVOT_P;
    config.Slot0.kI = PivotConstants.CLIMB_PIVOT_I;
    config.Slot0.kD = PivotConstants.CLIMB_PIVOT_D;

    config.Slot0.kS = PivotConstants.FF_CLIMB_PIVOT_S;
    config.Slot0.kV = PivotConstants.FF_CLIMB_PIVOT_G;
    config.Slot0.kA = PivotConstants.FF_CLIMB_PIVOT_V;

    climbMotor.getConfigurator().apply(config);

    climbMotorAppliedVoltage = climbMotor.getMotorVoltage();
    climbMotorAngle = climbMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, climbMotorAngle);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbPivotInputs inputs) {
    BaseStatusSignal.refreshAll(climbMotorAngle);
    inputs.position = climbMotorAngle.getValueAsDouble();
    inputs.currentVolts = climbMotorAppliedVoltage.getValueAsDouble();
  }

  @Override
  public void setClimbPivotPosition(double position) {
    climbMotor.setControl(motionMagicVoltage.withPosition(position));
  }

  @Override
  public void manualPivot(double percentSpeed) {
    climbMotor.set(percentSpeed);
  }

  @Override
  public double getClimbPivotPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  public double getVolts() {
    return climbMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setVolts(double volts) {
    climbMotor.setVoltage(volts);
  }
}

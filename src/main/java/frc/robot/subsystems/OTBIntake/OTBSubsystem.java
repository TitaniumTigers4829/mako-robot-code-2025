// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.OTBIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

public class OTBSubsystem extends SubsystemBase {
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;

  public OTBSubsystem() {
    // Configure pivot motor
    pivotMotor = new TalonFX(OTBSubsystemConstants.PIVOT_MOTOR_ID);
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = OTBSubsystemConstants.PIVOT_P;
    pivotConfig.Slot0.kI = OTBSubsystemConstants.PIVOT_I;
    pivotConfig.Slot0.kD = OTBSubsystemConstants.PIVOT_D;
    pivotMotor.getConfigurator().apply(pivotConfig);

    // Configure roller motor
    rollerMotor = new TalonFX(OTBSubsystemConstants.ROLLER_MOTOR_ID);
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kP = OTBSubsystemConstants.ROLLER_P;
    rollerConfig.Slot0.kI = OTBSubsystemConstants.ROLLER_I;
    rollerConfig.Slot0.kD = OTBSubsystemConstants.ROLLER_D;
    rollerMotor.getConfigurator().apply(rollerConfig);
  }
  /**
   * 
   * @param position Desired pivot position in rotations (0.0 to 1.0 corresponds to 0 to 360 degrees) r
   */
  public void setPivotPosition(double position) {
    pivotMotor.setControl(new PositionDutyCycle(position));
  }
/**
 * 
 * @param velocity Desired roller velocity in rotations per second  r/s
 */
  public void spinRoller(double velocity) {
    rollerMotor.setControl(new VelocityDutyCycle(velocity));
  }

  @Override
  public void periodic() {
    // Add any periodic updates here
  }
}

package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

public class PhysicalModule implements ModuleInterface {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC positionTorqueCurrentFOC =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveMotorAppliedVoltage;
  private final StatusSignal<Current> driveMotorCurrent;
  private final StatusSignal<Current> driveMotorTorque;
  private final StatusSignal<Double> driveMotorReference;

  private final StatusSignal<Angle> turnEncoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> turnEncoderVelocity;
  private final StatusSignal<Voltage> turnMotorAppliedVolts;
  private final StatusSignal<Current> turnMotorCurrent;
  private final StatusSignal<Current> turnMotorTorqueCurrent;
  private final StatusSignal<Double> turnMotorReference;

  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();

  public PhysicalModule(ModuleConfig moduleConfig) {
    driveMotor =
        new TalonFX(moduleConfig.driveMotorChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnMotor = new TalonFX(moduleConfig.turnMotorChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);
    turnEncoder =
        new CANcoder(moduleConfig.turnEncoderChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);

    turnEncoderConfig.MagnetSensor.MagnetOffset = -moduleConfig.angleZero();
    turnEncoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderReversed();
    turnEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.LOOP_TIME_SECONDS);

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = moduleConfig.driveReversed();
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimit = ModuleConstants.DRIVE_STATOR_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.ControlTimesyncFreqHz = 250;

    driveMotor.getConfigurator().apply(driveConfig, HardwareConstants.LOOP_TIME_SECONDS);

    turnConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.RotorToSensorRatio = ModuleConstants.TURN_GEAR_RATIO;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = moduleConfig.turnReversed();
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity =
        ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnConfig.MotorOutput.ControlTimesyncFreqHz = 250;
    turnMotor.getConfigurator().apply(turnConfig, HardwareConstants.LOOP_TIME_SECONDS);

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveMotorAppliedVoltage = driveMotor.getMotorVoltage();
    driveMotorCurrent = driveMotor.getSupplyCurrent();
    driveMotorReference = driveMotor.getClosedLoopReference();
    driveMotorTorque = driveMotor.getTorqueCurrent();

    turnEncoderAbsolutePosition = turnEncoder.getAbsolutePosition();
    turnEncoderVelocity = turnEncoder.getVelocity();
    turnMotorAppliedVolts = turnMotor.getMotorVoltage();
    turnMotorCurrent = turnMotor.getSupplyCurrent();
    turnMotorTorqueCurrent = turnMotor.getTorqueCurrent();
    turnMotorReference = turnMotor.getClosedLoopReference();

    driveMotor.setPosition(0.0);
    turnMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.CANIVORE_SIGNAL_FREQUENCY,
        drivePosition,
        turnEncoderAbsolutePosition,
        driveVelocity,
        driveMotorTorque,
        driveMotorReference,
        turnEncoderVelocity,
        turnMotorAppliedVolts,
        turnMotorCurrent,
        turnMotorReference,
        turnMotorTorqueCurrent);
    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
    turnEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        turnEncoderAbsolutePosition,
        driveVelocity,
        driveMotorTorque,
        driveMotorReference,
        turnEncoderVelocity,
        turnMotorAppliedVolts,
        turnMotorCurrent,
        turnMotorReference,
        turnMotorTorqueCurrent);

    inputs.isConnected =
        BaseStatusSignal.isAllGood(
            drivePosition,
            turnEncoderAbsolutePosition,
            driveVelocity,
            turnEncoderVelocity,
            driveMotorTorque,
            driveMotorReference,
            turnEncoderVelocity,
            turnMotorAppliedVolts,
            turnMotorCurrent,
            turnMotorReference,
            turnMotorTorqueCurrent);
    inputs.driveVelocity = driveVelocity.getValueAsDouble();
    inputs.drivePosition = -drivePosition.getValueAsDouble();
    inputs.driveDesiredPosition = driveMotorReference.getValueAsDouble();
    inputs.driveTorqueCurrent = driveMotorTorque.getValueAsDouble();
    inputs.driveError =
        Math.abs(driveMotorReference.getValueAsDouble() - driveMotorTorque.getValueAsDouble());

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnEncoderAbsolutePosition.getValueAsDouble());
    inputs.turnVelocity = turnEncoderVelocity.getValueAsDouble();
    inputs.turnDesiredPosition = turnMotorReference.getValueAsDouble();
    inputs.turnTorqueCurrent = turnMotorTorqueCurrent.getValueAsDouble();
    inputs.turnError =
        Math.abs(turnMotorReference.getValueAsDouble() - turnMotorTorqueCurrent.getValueAsDouble());
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    driveMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    turnMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setDriveCurrent(Current current) {
    driveMotor.setControl(currentOut.withOutput(current));
  }

  @Override
  public void setTurnCurrent(Current current) {
    turnMotor.setControl(currentOut.withOutput(current));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    driveMotor.setControl(
        velocityTorqueCurrentFOC
            .withVelocity(RotationsPerSecond.of(desiredDriveRPS))
            .withUseTimesync(true)
            .withOverrideCoastDurNeutral(true));
    turnMotor.setControl(
        positionTorqueCurrentFOC
            .withPosition(Rotations.of(desiredState.angle.getRotations()))
            .withUseTimesync(true)
            .withOverrideCoastDurNeutral(true));
  }

  public double getTurnRotations() {
    turnEncoder.getAbsolutePosition().refresh();
    return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble())
        .getRotations();
  }

  @Override
  public double getDrivePositionRadians() {
    drivePosition.refresh();
    return 2.0 * Math.PI * (drivePosition.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO);
  }

  @Override
  public void stopModule() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    driveMotor.getConfigurator().apply(driveConfig, 0.25);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    turnMotor.getConfigurator().apply(turnConfig, 0.25);
  }

  @Override
  public void setDriveFF(double kS, double kV, double kA) {
    driveConfig.Slot0.kS = kS;
    driveConfig.Slot0.kV = kV;
    driveConfig.Slot0.kA = kA;
    driveMotor.getConfigurator().apply(driveConfig, 0.25);
  }

  @Override
  public void setTurnFF(double kS, double kV, double kA) {
    turnConfig.Slot0.kS = kS;
    turnConfig.Slot0.kV = kV;
    turnConfig.Slot0.kA = kA;
    turnMotor.getConfigurator().apply(turnConfig, 0.25);
  }
}

package frc.robot.subsystems.algaePivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class PhysicalAlgaePivot implements AlgaePivotInterface {
  private final TalonFX algaeMotor;
  private final CANcoder algaeEncoder;
  private final TalonFXConfiguration algaeMotorConfig;
  private final CANcoderConfiguration algaeEncoderConfig;
  private final StatusSignal<Voltage> algaeVoltage;
  private final StatusSignal<AngularVelocity> algaeVelocity;
  private StatusSignal<Angle> algaeAngle;
  private final StatusSignal<Current> algaeSupplyCurrent;
  private final StatusSignal<Current> algaeStatorCurrent;
  private final MotionMagicVoltage mmPositionRequest;
  private double algaeTargetAngle;
  private final VoltageOut voltageOut;

  public PhysicalAlgaePivot() {
    algaeMotor = new TalonFX(AlgaePivotConstants.ALGAE_PIVOT_MOTOR_ID);
    algaeEncoder = new CANcoder(AlgaePivotConstants.ALGAE_ENCODER_MOTOR_ID);
    algaeMotorConfig = new TalonFXConfiguration();
    algaeEncoderConfig = new CANcoderConfiguration();
    mmPositionRequest = new MotionMagicVoltage(0);

    algaeVoltage = algaeMotor.getMotorVoltage();
    algaeVelocity = algaeMotor.getVelocity();
    algaeAngle = algaeEncoder.getAbsolutePosition();
    algaeSupplyCurrent = algaeMotor.getSupplyCurrent();
    algaeStatorCurrent = algaeMotor.getStatorCurrent();
    voltageOut = new VoltageOut(0);

    algaeEncoderConfig.MagnetSensor.MagnetOffset = -AlgaePivotConstants.ANGLE_ZERO;
    algaeEncoderConfig.MagnetSensor.SensorDirection = AlgaePivotConstants.ALGAE_ENCODER_REVERSED;
    algaeEncoder.getConfigurator().apply(algaeEncoderConfig, HardwareConstants.LOOP_TIME_SECONDS);

    algaeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeMotorConfig.MotorOutput.DutyCycleNeutralDeadband =
        HardwareConstants.MIN_DUTY_CYCLE_DEADBAND;

    algaeMotorConfig.Slot0.kP = AlgaePivotConstants.PIVOT_P;
    algaeMotorConfig.Slot0.kI = AlgaePivotConstants.PIVOT_I;
    algaeMotorConfig.Slot0.kD = AlgaePivotConstants.PIVOT_D;
    algaeMotorConfig.Slot0.kG = AlgaePivotConstants.PIVOT_G;
    algaeMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    algaeMotorConfig.MotionMagic.MotionMagicAcceleration =
        AlgaePivotConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
    algaeMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        AlgaePivotConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND;

    algaeMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    algaeMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    algaeMotorConfig.Feedback.FeedbackRemoteSensorID = algaeEncoder.getDeviceID();

    algaeMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaePivotConstants.MAX_ANGLE;
    algaeMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaePivotConstants.MIN_ANGLE;
    algaeMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    algaeMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, algaeAngle, algaeVelocity, algaeVoltage, algaeSupplyCurrent, algaeStatorCurrent);
  }

  @Override
  public void updateInputs(AlgaePivotInputs inputs) {
    inputs.algaeAngle = algaeAngle.getValueAsDouble();
    inputs.algaeVelocity = algaeVelocity.getValueAsDouble();
    inputs.algaeVoltage = algaeVoltage.getValueAsDouble();
  }

  @Override
  public void setAlgaeSpeed(double speed) {
    algaeMotor.set(speed);
  }

  @Override
  public void setAlgaeAngle(double angle) {
    algaeTargetAngle = angle;
    algaeMotor.setControl(mmPositionRequest.withPosition(angle));
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public double getAlgaeAngle() {
    algaeAngle.refresh();
    return algaeAngle.getValueAsDouble();
  }

  @Override
  public double getAlgaePivotTarget() {
    return algaeTargetAngle;
  }
}

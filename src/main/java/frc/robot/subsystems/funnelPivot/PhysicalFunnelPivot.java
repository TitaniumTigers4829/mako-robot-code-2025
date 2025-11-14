package frc.robot.subsystems.funnelPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.HardwareConstants;

public class PhysicalFunnelPivot implements FunnelPivotInterface {
  private final TalonFX funnelMotor;
  private final CANcoder funnelEncoder;
  private final TalonFXConfiguration funnelMotorConfig;
  private final CANcoderConfiguration funnelEncoderConfig;
  private final StatusSignal<Voltage> funnelVoltage;
  private final StatusSignal<AngularVelocity> funnelVelocity;
  private final StatusSignal<Angle> funnelAngle;
  private final StatusSignal<Current> funnelSupplyCurrent;
  private final StatusSignal<Current> funnelStatorCurrent;
  private final MotionMagicVoltage mmPositionRequest;
  private double funnelTargetAngle;
  private final VoltageOut voltageOut;

  private final Alert funnelMotorDisconnectAlert =
      new Alert("Funnel Motor Disconnect Alert", AlertType.kError);
  private final Alert funnelEncoderDiscconectAlert =
      new Alert("Funnel Encoder Disconnected Alert", AlertType.kError);

  public PhysicalFunnelPivot() {
    funnelMotor = new TalonFX(FunnelConstants.FUNNEL_PIVOT_MOTOR_ID);
    funnelEncoder =
        new CANcoder(FunnelConstants.FUNNEL_ENCODER_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    funnelMotorConfig = new TalonFXConfiguration();
    funnelEncoderConfig = new CANcoderConfiguration();
    mmPositionRequest = new MotionMagicVoltage(0);
    voltageOut = new VoltageOut(0);

    funnelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    funnelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    funnelMotorConfig.MotorOutput.DutyCycleNeutralDeadband =
        HardwareConstants.MIN_DUTY_CYCLE_DEADBAND;

    funnelMotorConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
    funnelMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND;

    funnelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    funnelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FunnelConstants.MAX_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FunnelConstants.MIN_ANGLE;
    funnelMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    funnelMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    funnelMotorConfig.Feedback.SensorToMechanismRatio = FunnelConstants.FUNNEL_GEAR_RATIO;

    funnelVoltage = funnelMotor.getMotorVoltage();
    funnelVelocity = funnelMotor.getVelocity();
    funnelAngle = funnelMotor.getPosition();
    funnelSupplyCurrent = funnelMotor.getSupplyCurrent();
    funnelStatorCurrent = funnelMotor.getStatorCurrent();

    funnelMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        funnelAngle,
        funnelVelocity,
        funnelVoltage,
        funnelSupplyCurrent,
        funnelStatorCurrent,
        funnelMotor.getClosedLoopReference());

    funnelMotor.optimizeBusUtilization();

    funnelMotor.getConfigurator().apply(funnelMotorConfig);
  }

  @Override
  public void updateInputs(FunnelPivotInputs inputs) {
    BaseStatusSignal.refreshAll(funnelAngle, funnelMotor.getClosedLoopReference());
    inputs.funnelAngle = funnelAngle.getValueAsDouble();
    inputs.funnelVelocity = funnelVelocity.getValueAsDouble();
    inputs.funnelVoltage = funnelVoltage.getValueAsDouble();
    inputs.closedLoop = funnelMotor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setFunnelSpeed(double speed) {
    funnelMotor.set(speed);
  }

  @Override
  public void setFunnelAngle(double angle) {
    funnelTargetAngle = angle;
    funnelMotor.setControl(mmPositionRequest.withPosition(angle));
  }

  @Override
  public void setFunnelVoltage(double voltage) {
    funnelMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public double getFunnelAngle() {
    funnelAngle.refresh();
    return funnelAngle.getValueAsDouble();
  }

  @Override
  public double getFunnelPivotTarget() {
    return funnelTargetAngle;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    funnelMotorConfig.Slot0.kP = kP;
    funnelMotorConfig.Slot0.kI = kI;
    funnelMotorConfig.Slot0.kD = kD;

    funnelMotor.getConfigurator().apply(funnelMotorConfig);
  }

  @Override
  public void checkAlerts() {
    boolean motorConnected =
        BaseStatusSignal.refreshAll(
                funnelVoltage, funnelVelocity, funnelSupplyCurrent, funnelStatorCurrent)
            .isOK();
    boolean encoderConnected = BaseStatusSignal.refreshAll(funnelAngle).isOK();

    funnelMotorDisconnectAlert.set(!motorConnected);
    funnelEncoderDiscconectAlert.set(!encoderConnected);
  }
}

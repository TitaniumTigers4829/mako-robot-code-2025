package frc.robot.subsystems.OTBIntake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.HardwareConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
  private final TalonFX leaderPivotMotor = new TalonFX(IntakeConstants.LEFT_INTAKE_PIVOT_MOTOR_ID);
  private final TalonFX followerPivotMotor =
      new TalonFX(IntakeConstants.RIGHT_INTAKE_PIVOT_MOTOR_ID);

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0);

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  public IntakeIOTalonFX() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    intakeConfig.CurrentLimits.StatorCurrentLimit = 0.0;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 0.0;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    intakeMotor.getConfigurator().apply(intakeConfig);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

    pivotConfig.CurrentLimits.StatorCurrentLimit = 0.0;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 0.0;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    pivotConfig.Slot0.kP = 0.0;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.0;
    pivotConfig.Slot0.kS = 0.0;
    pivotConfig.Slot0.kV = 0.0;
    pivotConfig.Slot0.kA = 0.0;
    pivotConfig.Slot0.kG = 0.0;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 0.0;

    // pivotConfig.Feedback.FeedbackRotorOffset = 0.0;
    // pivotConfig.Feedback.RotorToSensorRatio = 0.0;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leaderPivotMotor.setPosition(0);
    followerPivotMotor.setPosition(0);

    leaderPivotMotor.getConfigurator().apply(pivotConfig);
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    followerPivotMotor.getConfigurator().apply(pivotConfig);

    intakeVelocity = intakeMotor.getVelocity();
    pivotPosition = leaderPivotMotor.getRotorPosition();
    pivotVelocity = leaderPivotMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    inputs.pivotPosition = pivotPosition.getValueAsDouble();
    inputs.pivotVelocity = pivotVelocity.getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double position) {
    leaderPivotMotor.setControl(mmPositionRequest.withPosition(position));
    followerPivotMotor.setControl(mmPositionRequest.withPosition(position));
  }

  @Override
  public void setPivotSpeed(double speed) {
    leaderPivotMotor.set(speed);
    followerPivotMotor.set(speed);
  }

  @Override
  public double getPivotPosition() {
    leaderPivotMotor.getRotorPosition().refresh();
    return leaderPivotMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public double getIntakeSpeed() {
    intakeMotor.getVelocity().refresh();
    return intakeMotor.getVelocity().getValueAsDouble();
  }
}

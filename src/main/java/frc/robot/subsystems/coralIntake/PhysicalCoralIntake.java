package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants.HardwareConstants;

public class PhysicalCoralIntake implements CoralIntakeInterface {
  private final TalonFX coralIntakeMotor;
  private final DigitalInput innerCoralSensor;
  private final DigitalInput outerCoralSensor;

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Current> intakeSupplyCurrent;
  private final StatusSignal<Current> intakeStatorCurrent;
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<Temperature> intakeTemperatureCelsius;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Double> intakeDutyCycle;
  private final StatusSignal<Double> intakeReference;
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

  public PhysicalCoralIntake() {
    coralIntakeMotor = new TalonFX(CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID);
    innerCoralSensor = new DigitalInput(CoralIntakeConstants.INNER_CORAL_SENSOR_ID);
    outerCoralSensor = new DigitalInput(CoralIntakeConstants.OUTER_CORAL_SENSOR_ID);

    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_DUTY_CYCLE_DEADBAND;

    intakeConfig.CurrentLimits.StatorCurrentLimit = CoralIntakeConstants.INTAKE_STATOR_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_STATOR_LIMIT_ENABLE;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = CoralIntakeConstants.INTAKE_SUPPLY_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable =
        CoralIntakeConstants.INTAKE_SUPPLY_LIMIT_ENABLE;

    intakeConfig.Slot0.kP = CoralIntakeConstants.INTAKE_P;
    intakeConfig.Slot0.kS = CoralIntakeConstants.INTAKE_S;
    intakeConfig.Slot0.kV = CoralIntakeConstants.INTAKE_V;
    intakeConfig.Slot0.kA = CoralIntakeConstants.INTAKE_A;

    intakeConfig.MotionMagic.MotionMagicCruiseVelocity =
        CoralIntakeConstants.INTAKE_CRUISE_VELOCITY;
    intakeConfig.MotionMagic.MotionMagicAcceleration = CoralIntakeConstants.INTAKE_ACCELERATION;

    coralIntakeMotor.getConfigurator().apply(intakeConfig);

    intakeVelocity = coralIntakeMotor.getVelocity();
    intakePosition = coralIntakeMotor.getPosition();
    intakeStatorCurrent = coralIntakeMotor.getStatorCurrent();
    intakeSupplyCurrent = coralIntakeMotor.getSupplyCurrent();
    intakeTemperatureCelsius = coralIntakeMotor.getDeviceTemp();
    intakeAppliedVolts = coralIntakeMotor.getMotorVoltage();
    intakeDutyCycle = coralIntakeMotor.getDutyCycle();
    intakeReference = coralIntakeMotor.getClosedLoopReference();
    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.RIO_SIGNAL_FREQUENCY,
        intakeVelocity,
        intakePosition,
        intakeStatorCurrent,
        intakeStatorCurrent,
        intakeSupplyCurrent,
        intakeTemperatureCelsius,
        intakeTemperatureCelsius,
        intakeAppliedVolts,
        intakeDutyCycle,
        intakeReference);
    coralIntakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralIntakeInputs intakeInputs) {
    BaseStatusSignal.refreshAll(intakeVelocity, intakeReference);
    intakeInputs.isMotorConnected =
        BaseStatusSignal.isAllGood(
            intakeVelocity,
            intakeStatorCurrent,
            intakeSupplyCurrent,
            intakeTemperatureCelsius,
            intakePosition,
            intakeAppliedVolts,
            intakeDutyCycle,
            intakeReference);
    intakeInputs.isInnerSensorConnected =
        innerCoralSensor.getChannel() == CoralIntakeConstants.INNER_CORAL_SENSOR_ID;
    intakeInputs.isOuterSensorConnected =
        outerCoralSensor.getChannel() == CoralIntakeConstants.OUTER_CORAL_SENSOR_ID;
    intakeInputs.intakeVelocity = intakeVelocity.getValueAsDouble();
    intakeInputs.intakeStatorCurrentAmps = intakeStatorCurrent.getValueAsDouble();
    intakeInputs.intakeTemp = intakeTemperatureCelsius.getValueAsDouble();
    intakeInputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    intakeInputs.intakePosition = intakePosition.getValueAsDouble();
    intakeInputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
    intakeInputs.hasCoral = !innerCoralSensor.get(); // true if coral is sensed
    intakeInputs.hasControl = !outerCoralSensor.get();
    intakeInputs.intakeDutyCycle = intakeDutyCycle.getValueAsDouble();
    intakeInputs.intakeReference = intakeReference.getValueAsDouble();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    coralIntakeMotor.set(speed);
  }

  @Override
  public void setIntakeVelocity(double velocity) {
    coralIntakeMotor.setControl(velocityVoltage.withVelocity(velocity / 60.0));
  }

  @Override
  public boolean hasCoral() {
    return !innerCoralSensor.get();
  }

  @Override
  public boolean hasControl() {
    return !outerCoralSensor.get();
  }

  @Override
  public double getIntakeSpeed() {
    coralIntakeMotor.getVelocity().refresh();
    return coralIntakeMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    coralIntakeMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPID(double P) {
    intakeConfig.Slot0.kP = P;
    coralIntakeMotor.getConfigurator().apply(intakeConfig);
  }
  public class PIDController{

    public double kP = 5.0;
    public double ki = 0.0;
    public double kd = 0.0;
     

    MotorControllerRPM rpmController = new MotorControllerRPM(kP, ki, kd);
    rpmController.setTargetRPM(1500);
    
    
    
  }
}

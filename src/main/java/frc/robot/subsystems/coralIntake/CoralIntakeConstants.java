package frc.robot.subsystems.coralIntake;

public final class CoralIntakeConstants {
  public static final int CORAL_INTAKE_MOTOR_ID = 30;
  public static final int INNER_CORAL_SENSOR_ID = 1;
  public static final int OUTER_CORAL_SENSOR_ID = 0;

  public static final double WAITING_INTAKE_SPEED = 1500;
  public static final double INGEST_SPEED = 700;
  public static final double EJECT_SPEED = 3000;
  public static final double REVERSE_INTAKE_SPEED = -200;
  public static final double NEUTRAL_INTAKE_SPEED = 0.0;

  /** In seconds */
  public static final double SENSOR_DEBOUNCE_TIME = 0.04;

  // PID and feedforward constants
  public static final double INTAKE_P = 0.3;
  public static final double INTAKE_S = 0.566927529597441;
  public static final double INTAKE_V = 0.098275156522801;
  public static final double INTAKE_A = 0.013968715343075;

  // Motion magic
  public static final double INTAKE_CRUISE_VELOCITY = 50;
  public static final double INTAKE_ACCELERATION = 150;

  // the max amount of stator and supply current allowed in the motor, respectively
  public static final double INTAKE_STATOR_LIMIT = 80.0;
  public static final double INTAKE_SUPPLY_LIMIT = 0.0;
  // Enables stator and supply current limits, respectively
  public static final boolean INTAKE_STATOR_LIMIT_ENABLE = true;
  public static final boolean INTAKE_SUPPLY_LIMIT_ENABLE = false;
}

package frc.robot.subsystems.OTBIntake;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final int PIVOT_MOTOR_ID = 10;
  public static final int INTAKE_ROLLERS_MOTOR_ID = 11;
  public static final int INDEXER_MOTOR_ID = 12;

  // Optional sensor: DIO, CANcoder, beam-break, etc.
  // Use a real DIO channel (0–9 typical on roboRIO).
  public static final int NOTE_SENSOR_DIO = 0; // set to your real channel

  // =========================
  // Mechanics & Units
  // =========================
  /** Motor-to-arm gear reduction. Example: 100.0 means 100 motor turns = 1 arm turn. */
  public static final double PIVOT_GEAR_RATIO = 100.0;

  /** Zero offset of the arm (radians) if your encoder zero isn't physically at STOW. */
  public static final double PIVOT_ZERO_OFFSET_RAD = 0.0;

  // Choose ONE position unit for your codebase. Here we store SETPOINTS IN DEGREES for readability:
  public static final double ANGLE_STOW_DEG = -15.0; // inside frame perimeter
  public static final double ANGLE_INTAKE_DEG = 35.0; // down to grab
  public static final double ANGLE_FEED_DEG = 65.0; // up to handoff/index

  /** Acceptable on-target error (degrees). */
  public static final double PIVOT_TOLERANCE_DEG = 2.0;

  // =========================
  // Roller Speeds (percent)
  // =========================
  /** Intake rollers forward (grab). */
  public static final double INTAKE_SPEED = 0.80; // [-1,1]

  /** Intake rollers reverse (spit). */
  public static final double OUTTAKE_SPEED = -0.60; // [-1,1]

  /** Indexer feed forward. */
  public static final double INDEXER_FEED_SPEED = 0.70;

  /** Indexer reverse. */
  public static final double INDEXER_REVERSE_SPEED = -0.50;

  /** Neutral/hold speeds (usually 0 for percent-output). */
  public static final double INTAKE_NEUTRAL_SPEED = 0.0;

  public static final double INDEXER_NEUTRAL_SPEED = 0.0;

  // =========================
  // Pivot control (PID + FF)
  // =========================
  // If you’re using Trapezoid-Profiled PID for pivots:
  public static final double PIVOT_kP = 8.0;
  public static final double PIVOT_kI = 0.0;
  public static final double PIVOT_kD = 0.2;

  /** Motion limits in deg/s and deg/s^2 (convert to radians in code if needed). */
  public static final double PIVOT_MAX_VEL_DEG_PER_S = 120.0;

  public static final double PIVOT_MAX_ACC_DEG_PER_S2 = 250.0;

  /** Arm feedforward (tune on robot): V = kS*sign(vel)+kG*cos(theta)+kV*vel + kA*acc */
  public static final double PIVOT_kS = 0.20;

  public static final double PIVOT_kG = 0.90;
  public static final double PIVOT_kV = 2.20;
  public static final double PIVOT_kA = 0.05;

  // =========================
  // Safety / Limits
  // =========================
  /** Soft limits in degrees (mechanical safe range). */
  public static final boolean ENABLE_SOFT_LIMITS = true;

  public static final double PIVOT_MIN_DEG = -30.0;
  public static final double PIVOT_MAX_DEG = 80.0;

  // Current limits (adjust for your motors and breakers)
  public static final int PIVOT_STATOR_LIMIT_A = 60;
  public static final int PIVOT_SUPPLY_LIMIT_A = 40;
  public static final boolean PIVOT_STATOR_EN = true;
  public static final boolean PIVOT_SUPPLY_EN = true;

  public static final int INTAKE_STATOR_LIMIT_A = 60;
  public static final int INTAKE_SUPPLY_LIMIT_A = 40;
  public static final boolean INTAKE_STATOR_EN = true;
  public static final boolean INTAKE_SUPPLY_EN = true;

  public static final int INDEXER_STATOR_LIMIT_A = 40;
  public static final int INDEXER_SUPPLY_LIMIT_A = 30;
  public static final boolean INDEXER_STATOR_EN = true;
  public static final boolean INDEXER_SUPPLY_EN = true;
}

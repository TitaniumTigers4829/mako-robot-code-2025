package frc.robot.subsystems.OTBIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  // ----- Intake (roller) sim -----
  private static final double kIntakeGearing = 1.0;
  private static final double kIntakeMoi = 0.0005; // kg m^2 (tune)
  private final DCMotorSim intakeSim =
      new DCMotorSim(DCMotor.getFalcon500(1), kIntakeGearing, kIntakeMoi);
  private double intakeVoltsCmd = 0.0;

  // ----- Pivot (arm) sim -----
  // Mechanism parameters (tune to your robot)
  private static final double kArmLengthMeters = 0.35; // arm COM distance
  private static final double kArmMoi = 0.25; // kg m^2
  private static final double kMinAngleRad = Units.degreesToRadians(IntakeConstants.PIVOT_MIN_DEG);
  private static final double kMaxAngleRad = Units.degreesToRadians(IntakeConstants.PIVOT_MAX_DEG);

  // Gear: motor rotor to arm (mechanism) — should match your real gearbox (e.g., 100:1)
  private static final double kGearRatio = IntakeConstants.PIVOT_GEAR_RATIO;

  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(2),
          kGearRatio, // rotor:arm
          kArmMoi,
          kArmLengthMeters,
          kMinAngleRad,
          kMaxAngleRad,
          true, // simulate gravity
          Units.degreesToRadians(IntakeConstants.ANGLE_STOW_DEG)); // start

  // Motion/profile + ff (use reasonable starters; tune later)
  private final Constraints pivotConstraints =
      new Constraints(
          Units.degreesToRadians(IntakeConstants.PIVOT_MAX_VEL_DEG_PER_S),
          Units.degreesToRadians(IntakeConstants.PIVOT_MAX_ACC_DEG_PER_S2));
  private final ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          IntakeConstants.PIVOT_kP,
          IntakeConstants.PIVOT_kI,
          IntakeConstants.PIVOT_kD,
          pivotConstraints);
  private final ArmFeedforward pivotFF =
      new ArmFeedforward(
          IntakeConstants.PIVOT_kS, IntakeConstants.PIVOT_kG,
          IntakeConstants.PIVOT_kV, IntakeConstants.PIVOT_kA);
  private double pivotVoltsCmd = 0.0;

  public IntakeIOSim() {
    // Nothing heavy here; sim state advances in updateInputs()
  }

  // ----- Helper conversions -----
  private static double mechRotsToRad(double mechRots) {
    return mechRots * 2.0 * Math.PI;
  }

  private static double radToMechRots(double rad) {
    return rad / (2.0 * Math.PI);
  }

  // ===== IO API =====
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Advance physics by 20 ms
    final double dt = 0.02;
    intakeSim.setInputVoltage(intakeVoltsCmd);
    pivotSim.setInputVoltage(pivotVoltsCmd);
    intakeSim.update(dt);
    pivotSim.update(dt);

    // Fill intake signals
    inputs.intakeAppliedVolts = intakeVoltsCmd;
    inputs.intakeVelocity = intakeSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI); // rot/s
    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.intakeTemp = 0.0; // no thermal model

    // Fill pivot signals
    inputs.pivotAppliedVolts = pivotVoltsCmd;
    inputs.pivotPosition = radToMechRots(pivotSim.getAngleRads()); // mech rotations
    inputs.pivotVelocity = radToMechRots(pivotSim.getVelocityRadPerSec()); // rot/s
    inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotTemp = 0.0;

    // Mark connected in sim
    inputs.isConnected = RobotBase.isSimulation();
  }

  /** Closed-loop to a **mechanism** position (rotations of the arm). */
  @Override
  public void setPivotPosition(double mechRotsGoal) {
    final double goalRad = mechRotsToRad(mechRotsGoal);
    final double measRad = pivotSim.getAngleRads();
    // Profiled PID: calculate(measurement, goal)
    final double pidVolts = pivotPID.calculate(measRad, goalRad);
    final double ffVolts =
        pivotFF.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);
    pivotVoltsCmd = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
  }

  /** Open-loop intake as percent output [-1,1]. */
  @Override
  public void setIntakeSpeed(double percent) {
    intakeVoltsCmd = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
  }

  /** Open-loop pivot as percent output [-1,1] (for jog). */
  @Override
  public void setPivotSpeed(double percent) {
    pivotVoltsCmd = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
  }

  @Override
  public double getIntakeSpeed() {
    return intakeSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI); // rot/s
  }

  @Override
  public double getPivotPosition() {
    return radToMechRots(pivotSim.getAngleRads()); // mech rotations
  }
}

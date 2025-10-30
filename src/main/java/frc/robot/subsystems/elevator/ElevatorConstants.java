// copyrightIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int ELEVATOR_LEADER_MOTOR_ID = 0;
  public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 62;

  public static final double ELEVATOR_P = 10;
  public static final double ELEVATOR_I = 0;
  public static final double ELEVATOR_D = 0.0;
  public static final double ELEVATOR_S = 0.20;
  public static final double ELEVATOR_V = .8;
  public static final double ELEVATOR_A = 0.057759439794094;
  public static final double ELEVATOR_G = -0.35;

  public static final double DRUM_RADIUS = 1;
  public static final double ELEVATOR_GEAR_RATIO = 4.8; // 29
  public static final double ELEVATOR_CARRIAGE_MASS = 10;
  public static final double MIN_HEIGHT = 0;
  public static final double MAX_HEIGHT = 3;
  public static final double INCLINE_ANGLE_RADIANS = Units.degreesToRadians(8);
  public static final boolean SIMULATE_GRAVITY = true;

  public static final double STATOR_CURRENT_LIMIT = 100;
  public static final double SUPPLY_CURRENT_LIMIT = 0;
  public static final boolean STATOR_CURRENT_LIMIT_ENABLE = true;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = false;

  public static final double MOTION_MAGIC_MAX_ACCELERATION = 80; // prev: 160
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 50; // prev: 100

  public static final double ELEVATOR_ERROR_TOLERANCE = 0.08;

  // limit
  public static final double LIMIT = 0.0;
  public static final boolean LIMIT_ENABLE = true;
  public static final double REVERSE_LIMIT = -9.3;
  public static final boolean REVERSE_LIMIT_ENABLE = true;

  // Elevator setpoints
  public enum ElevatorSetpoints {
    L1(-2.50),
    L2(-2.8 + .00004829),
    L3(-4.517),
    L4(-7.55), // -9.2
    FEEDER(-0.3);

    private final double position;

    ElevatorSetpoints(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }
}

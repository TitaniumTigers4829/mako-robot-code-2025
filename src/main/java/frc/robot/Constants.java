package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  private static RobotType robotType = RobotType.COMP_ROBOT;

  public static final boolean tuningMode = false;

  /**
   * Gets if the robot type is valid, if not it will default to COMP_ROBOT
   *
   * @return the currently used RobotType
   */
  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    // if (RobotBase.isReal() && robotType == RobotType.SIM_ROBOT) {
    //   new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
    //       .set(true);
    //   robotType = RobotType.COMP_ROBOT;
    // }
    return robotType;
  }

  /**
   * Gets the mode of the robot based on the RobotType and the state of {@link RobotBase}, if the
   * robot isn't real but is also not the SIM_ROBOT, it will set the currently used mode to REPLAY
   *
   * @return the currently used Mode
   */
  public static Mode getMode() {
    return switch (robotType) {
      case DEV_ROBOT, COMP_ROBOT, SWERVE_ROBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_ROBOT -> Mode.SIM;
    };
  }

  /** An enum to select the robot's mode. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** An enum to select the currently used robot. */
  public enum RobotType {
    SIM_ROBOT,
    DEV_ROBOT,
    COMP_ROBOT,
    SWERVE_ROBOT
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotType.SIM_ROBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotType.DEV_ROBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  /**
   * This is where we place constants related to hardware on a robot that aren't specific to any
   * singular subsystem.
   */
  public static final class HardwareConstants {
    public static final double LOOP_TIME_SECONDS = 0.02;

    public static final double RIO_SIGNAL_FREQUENCY = 50;
    public static final double CANIVORE_SIGNAL_FREQUENCY = 250;

    public static final String CANIVORE_CAN_BUS_STRING = "carnivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";

    /**
     * For some reason, falcons normally have a deadband threshold of 4%. This is incredibly high!
     * It makes it very hard to do precise movements, so with this constant we set the threshold to
     * the lowest possible value.
     */
    public static final double MIN_DUTY_CYCLE_DEADBAND = 0.001;

    public static final double MIN_TORQUE_DEADBAND = 5.0;

    public static final int HIGH_THREAD_PRIORITY = 99;
    public static final int LOW_THREAD_PRIORITY = 1;
  }

  /**
   * This is where constants used to describe the game's field go. This will have the dimensions of
   * the field, but also the coordinates of obstacles, game pieces, or other places of interest.
   */
  public static final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(690);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317);

    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2);

    public static final double REEF_LEVEL_ONE_Z = Units.inchesToMeters(18);
    public static final double REEF_LEVEL_TWO_Z = Units.inchesToMeters(31.875);
    public static final double REEF_LEVEL_THREE_Z = Units.inchesToMeters(47.625);
    public static final double REEF_LEVEL_FOUR_Z = Units.inchesToMeters(72);

    public static final double CORAL_STATION_Z = Units.inchesToMeters(37.5);

    public static final double PROCESSOR_Z = Units.inchesToMeters(7);

    /* The meaning behind the numbering of the reefs is that it starts from the reef closest to the driver station (Blue side)
    or the reef closer to the robots (Red side) and goes clockwise. Now thinking about it, I should probably find a better way
    to number these reefs so they have some sense on how they're numbered, but that's for other members to decide.
    */
    public static final Pose2d BLUE_REEF_ONE =
        new Pose2d(
            Units.inchesToMeters(155.975),
            Units.inchesToMeters(152.032),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d BLUE_REEF_TWO =
        new Pose2d(
            Units.inchesToMeters(156.02),
            Units.inchesToMeters(164.97),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d BLUE_REEF_THREE =
        new Pose2d(
            Units.inchesToMeters(160.776),
            Units.inchesToMeters(173.226),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d BLUE_REEF_FOUR =
        new Pose2d(
            Units.inchesToMeters(172.003),
            Units.inchesToMeters(179.655),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d BLUE_REEF_FIVE =
        new Pose2d(
            Units.inchesToMeters(181.511),
            Units.inchesToMeters(179.655),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d BLUE_REEF_SIX =
        new Pose2d(
            Units.inchesToMeters(192.693),
            Units.inchesToMeters(173.266),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d BLUE_REEF_SEVEN =
        new Pose2d(
            Units.inchesToMeters(197.515),
            Units.inchesToMeters(165.362),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_REEF_EIGHT =
        new Pose2d(
            Units.inchesToMeters(197.47),
            Units.inchesToMeters(152.03),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_REEF_NINE =
        new Pose2d(
            Units.inchesToMeters(192.714),
            Units.inchesToMeters(143.774),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d BLUE_REEF_TEN =
        new Pose2d(
            Units.inchesToMeters(181.487),
            Units.inchesToMeters(137.345),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d BLUE_REEF_ELEVEN =
        new Pose2d(
            Units.inchesToMeters(137.345),
            Units.inchesToMeters(137.345),
            new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d BLUE_REEF_TWELEVE =
        new Pose2d(3.70, 2.83, Rotation2d.fromDegrees(62.50));
    // Units.inchesToMeters(160.797),
    // Units.inchesToMeters(143.814),
    // new Rotation2d(Units.degreesToRadians(240)));

    public static final Pose2d RED_REEF_ONE =
        new Pose2d(
            Units.inchesToMeters(493.365),
            Units.inchesToMeters(152.032),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_REEF_TWO =
        new Pose2d(
            Units.inchesToMeters(493.41),
            Units.inchesToMeters(164.97),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_REEF_THREE =
        new Pose2d(
            Units.inchesToMeters(498.156),
            Units.inchesToMeters(173.226),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d RED_REEF_FOUR =
        new Pose2d(
            Units.inchesToMeters(509.383),
            Units.inchesToMeters(179.655),
            new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d RED_REEF_FIVE =
        new Pose2d(
            Units.inchesToMeters(518.901),
            Units.inchesToMeters(179.693),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RED_REEF_SIX =
        new Pose2d(
            Units.inchesToMeters(530.083),
            Units.inchesToMeters(173.186),
            new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RED_REEF_SEVEN =
        new Pose2d(
            Units.inchesToMeters(534.895),
            Units.inchesToMeters(164.968),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d RED_REEF_EIGHT =
        new Pose2d(
            Units.inchesToMeters(534.85),
            Units.inchesToMeters(152.03),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d RED_REEF_NINE =
        new Pose2d(
            Units.inchesToMeters(530.104),
            Units.inchesToMeters(143.774),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d RED_REEF_TEN =
        new Pose2d(
            Units.inchesToMeters(518.877),
            Units.inchesToMeters(137.345),
            new Rotation2d(Units.degreesToRadians(300)));
    public static final Pose2d RED_REEF_ELEVEN =
        new Pose2d(
            Units.inchesToMeters(509.359),
            Units.inchesToMeters(137.307),
            new Rotation2d(Units.degreesToRadians(240)));
    public static final Pose2d RED_REEF_TWELEVE =
        new Pose2d(
            Units.inchesToMeters(498.177),
            Units.inchesToMeters(143.814),
            new Rotation2d(Units.degreesToRadians(240)));

    public static final Pose2d BLUE_LEFT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(25.8),
            new Rotation2d(Units.degreesToRadians(54)));
    public static final Pose2d BLUE_RIGHT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(291.2),
            new Rotation2d(Units.degreesToRadians(306)));
    public static final Pose2d RED_LEFT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(657.37),
            Units.inchesToMeters(291.2),
            new Rotation2d(Units.degreesToRadians(234)));
    public static final Pose2d RED_RIGHT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(657.37),
            Units.inchesToMeters(25.8),
            new Rotation2d(Units.degreesToRadians(126)));

    public static final Pose2d BLUE_PROCESSOR =
        new Pose2d(
            Units.inchesToMeters(235.73),
            Units.inchesToMeters(0),
            new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d RED_PROCESSOR =
        new Pose2d(
            Units.inchesToMeters(455.15),
            Units.inchesToMeters(317),
            new Rotation2d(Units.degreesToRadians(270)));

    // We wont be able to pick these game pieces up
    public static final Pose2d BLUE_LEFT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(47.967),
            Units.inchesToMeters(86.50),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_MIDDLE_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(47.967),
            Units.inchesToMeters(158.50),
            new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_RIGHT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(47.967),
            Units.inchesToMeters(230.50),
            new Rotation2d(Units.degreesToRadians(0)));

    public static final Pose2d RED_LEFT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(642.903),
            Units.inchesToMeters(230.50),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_MIDDLE_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(642.903),
            Units.inchesToMeters(158.50),
            new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_RIGHT_PRE_PLACED_POINT =
        new Pose2d(
            Units.inchesToMeters(642.903),
            Units.inchesToMeters(86.50),
            new Rotation2d(Units.degreesToRadians(180)));

    public static final double RED_REEF_PLACE_X = 2;
    public static final double RED_REEF_PLACE_Y = 2;
    public static final Rotation2d RED_REEF_ROTATION = Rotation2d.fromDegrees(-90);

    public static final double RED_PROCESSOR_PLACE_X = 0;
    public static final double RED_PROCESSOR_PLACE_Y = 0;
    public static final Rotation2d RED_PROCESSOR_ROTATION = Rotation2d.fromDegrees(-90);

    public static final double RED_FEEDER_STATION_PLACE_X = 0;
    public static final double RED_FEEDER_STATION_PLACE_Y = 0;
    public static final Rotation2d RED_FEEDER_STATION_ROTATION = Rotation2d.fromDegrees(-90);
  }

  /** This is where we place constants related to Auto and any auto-related features */
  public static final class AutoConstants {

    // Different Pre-defined Auto Routines
    public static final String Y_ONE_METER_AUTO = "Y-One-Meter-Test";
    public static final String X_ONE_METER_AUTO = "X-One-Meter-Test";
    public static final String BLUE_LEFT_TWO_CORAL_AUTO_ROUTINE =
        "Blue-Left-Two-Coral-Auto-Routine";
    public static final String BLUE_MID_TWO_CORAL_AUTO_ROUTINE = "Blue-Mid-Two-Coral-Auto-Routine";
    public static final String BLUE_RIGHT_TWO_CORAL_AUTO_ROUTINE =
        "Blue-Right-Two-Coral-Auto-Routine";
    public static final String BLUE_LEFT_THREE_CORAL_AUTO_ROUTINE =
        "Blue-Left-Three-Coral-Auto-Routine";
    public static final String BLUE_MID_THREE_CORAL_AUTO_ROUTINE =
        "Blue-Mid-Three-Coral-Auto-Routine";
    public static final String BLUE_RIGHT_THREE_CORAL_AUTO_ROUTINE =
        "Blue-Right-Three-Coral-Auto-Routine";
    public static final String BLUE_LEFT_FOUR_CORAL_AUTO_ROUTINE =
        "Blue-Left-Four-Coral-Auto-Routine";
    public static final String BLUE_MID_FOUR_CORAL_AUTO_ROUTINE =
        "Blue-Mid-Four-Coral-Auto-Routine";
    public static final String BLUE_RIGHT_FOUR_CORAL_AUTO_ROUTINE =
        "Blue-Right-Four-Coral-Auto-Routine";
    public static final String SIMPLE_REPULSOR_AUTO = "Simple-Repulsor-Auto";
    public static final String TWO_CORAL_REPULSOR_AUTO = "Two-Coral-Repulsor-Auto";
    // This does not exist yet :(

    // All Trajectories are created on the blue alliance and are flipped in the code
    // Right and Left are always from the perspective of the driver station
    // letters represent nodes on the coral according to FIRST's official system:

    //  Left      /-----------------------|
    //          /        K--J             |
    //         |       L------I           |
    // Driver  |     A----------H         |
    // Station |     B----------G         |
    //         |       C------F           |
    //          \        D--E             |
    //  Right     \-----------------------|

    // Example 1 Meter Trajectory

    // Right Start
    public static final String BLUE_RIGHT_START_TO_E_TRAJECTORY =
        "BlueTrajectories/Right-Start-to-E";
    public static final String BLUE_RIGHT_START_TO_F_TRAJECTORY =
        "BlueTrajectories/Right-Start-to-F";
    public static final String BLUE_RIGHT_START_TO_G_TRAJECTORY =
        "BlueTrajectories/Right-Start-to-G";
    // Mid Start
    public static final String BLUE_MID_START_TO_E_TRAJECTORY = "BlueTrajectories/Mid-Start-to-E";
    public static final String BLUE_MID_START_TO_F_TRAJECTORY = "BlueTrajectories/Mid-Start-to-F";
    public static final String BLUE_MID_START_TO_G_TRAJECTORY = "BlueTrajectories/Mid-Start-to-G";
    public static final String BLUE_MID_START_TO_H_TRAJECTORY = "BlueTrajectories/Mid-Start-to-H";
    public static final String BLUE_MID_START_TO_I_TRAJECTORY = "BlueTrajectories/Mid-Start-to-I";
    public static final String BLUE_MID_START_TO_J_TRAJECTORY = "BlueTrajectories/Mid-Start-to-J";
    // Left Start
    public static final String BLUE_LEFT_START_TO_I_TRAJECTORY = "BlueTrajectories/Left-Start-to-I";
    public static final String BLUE_LEFT_START_TO_H_TRAJECTORY = "BlueTrajectories/Left-Start-to-H";
    public static final String BLUE_LEFT_START_TO_J_TRAJECTORY = "BlueTrajectories/Left-Start-to-J";

    // tuning 1 meter
    public static final String X_ONE_METER_TRAJECTORY = "BlueTrajectories/X1METER";
    public static final String Y_ONE_METER_TRAJECTORY = "BlueTrajectories/Y1METER";

    // Reef to Right Pickup
    public static final String BLUE_A_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/A-to-Right-Pickup";
    public static final String BLUE_B_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/B-to-Right-Pickup";
    public static final String BLUE_C_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/C-to-Right-Pickup";
    public static final String BLUE_D_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/D-to-Right-Pickup";
    public static final String BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/E-to-Right-Pickup";
    public static final String BLUE_F_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/F-to-Right-Pickup";
    public static final String BLUE_G_TO_RIGHT_PICKUP_TRAJECTORY =
        "BlueTrajectories/G-to-Right-Pickup";
    // Reef to Left Pickup
    public static final String BLUE_H_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/H-to-Left-Pickup";
    public static final String BLUE_I_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/I-to-Left-Pickup";
    public static final String BLUE_J_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/J-to-Left-Pickup";
    public static final String BLUE_K_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/K-to-Left-Pickup";
    public static final String BLUE_L_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/L-to-Left-Pickup";
    public static final String BLUE_A_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/A-to-Left-Pickup";
    public static final String BLUE_B_TO_LEFT_PICKUP_TRAJECTORY =
        "BlueTrajectories/B-to-Left-Pickup";
    // Right Pickup to Reef
    public static final String BLUE_RIGHT_PICKUP_TO_A_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-A";
    public static final String BLUE_RIGHT_PICKUP_TO_B_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-B";
    public static final String BLUE_RIGHT_PICKUP_TO_C_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-C";
    public static final String BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-D";
    public static final String BLUE_RIGHT_PICKUP_TO_E_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-E";
    public static final String BLUE_RIGHT_PICKUP_TO_F_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-F";
    public static final String BLUE_RIGHT_PICKUP_TO_G_TRAJECTORY =
        "BlueTrajectories/Right-Pickup-to-G";
    // Left Pickup to Reef
    public static final String BLUE_LEFT_PICKUP_TO_H_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-H";
    public static final String BLUE_LEFT_PICKUP_TO_I_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-I";
    public static final String BLUE_LEFT_PICKUP_TO_J_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-J";
    public static final String BLUE_LEFT_PICKUP_TO_K_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-K";
    public static final String BLUE_LEFT_PICKUP_TO_L_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-L";
    public static final String BLUE_LEFT_PICKUP_TO_A_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-A";
    public static final String BLUE_LEFT_PICKUP_TO_B_TRAJECTORY =
        "BlueTrajectories/Left-Pickup-to-B";

    // Different Pre-defined Auto Routines
    public static final String RED_RIGHT_TWO_CORAL_AUTO_ROUTINE =
        "Red-Right-Two-Coral-Auto-Routine";
    public static final String RED_MID_TWO_CORAL_AUTO_ROUTINE = "Red-Mid-Two-Coral-Auto-Routine";
    public static final String RED_LEFT_TWO_CORAL_AUTO_ROUTINE = "Red-Left-Two-Coral-Auto-Routine";
    public static final String RED_RIGHT_THREE_CORAL_AUTO_ROUTINE =
        "Red-Right-Three-Coral-Auto-Routine";
    public static final String RED_MID_THREE_CORAL_AUTO_ROUTINE =
        "Red-Mid-Three-Coral-Auto-Routine";
    public static final String RED_LEFT_THREE_CORAL_AUTO_ROUTINE =
        "Red-Left-Three-Coral-Auto-Routine";
    public static final String RED_RIGHT_FOUR_CORAL_AUTO_ROUTINE =
        "Red-Right-Four-Coral-Auto-Routine";
    public static final String RED_MID_FOUR_CORAL_AUTO_ROUTINE = "Red-Mid-Four-Coral-Auto-Routine";
    public static final String RED_LEFT_FOUR_CORAL_AUTO_ROUTINE =
        "Red-Left-Four-Coral-Auto-Routine";
    public static final Pose2d BLUE_LEFT_CORAL_STATION =
        new Pose2d(
            Units.inchesToMeters(33.51),
            Units.inchesToMeters(25.8),
            new Rotation2d(Units.degreesToRadians(54)));
    // Right Start
    public static final String RED_RIGHT_START_TO_E_TRAJECTORY = "RedTrajectories/Right-Start-to-E";
    public static final String RED_RIGHT_START_TO_F_TRAJECTORY = "RedTrajectories/Right-Start-to-F";
    public static final String RED_RIGHT_START_TO_G_TRAJECTORY = "RedTrajectories/Right-Start-to-G";
    // Mid Start
    public static final String RED_MID_START_TO_E_TRAJECTORY = "RedTrajectories/Mid-Start-to-E";
    public static final String RED_MID_START_TO_F_TRAJECTORY = "RedTrajectories/Mid-Start-to-F";
    public static final String RED_MID_START_TO_G_TRAJECTORY = "RedTrajectories/Mid-Start-to-G";
    public static final String RED_MID_START_TO_H_TRAJECTORY = "RedTrajectories/Mid-Start-to-H";
    public static final String RED_MID_START_TO_I_TRAJECTORY = "RedTrajectories/Mid-Start-to-I";
    public static final String RED_MID_START_TO_J_TRAJECTORY = "RedTrajectories/Mid-Start-to-J";
    // Left Start
    public static final String RED_LEFT_START_TO_I_TRAJECTORY = "RedTrajectories/Left-Start-to-I";
    public static final String RED_LEFT_START_TO_H_TRAJECTORY = "RedTrajectories/Left-Start-to-H";
    public static final String RED_LEFT_START_TO_J_TRAJECTORY = "RedTrajectories/Left-Start-to-J";

    // Reef to Right Pickup
    public static final String RED_A_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/A-to-Right-Pickup";
    public static final String RED_B_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/B-to-Right-Pickup";
    public static final String RED_C_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/C-to-Right-Pickup";
    public static final String RED_D_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/D-to-Right-Pickup";
    public static final String RED_E_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/E-to-Right-Pickup";
    public static final String RED_F_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/F-to-Right-Pickup";
    public static final String RED_G_TO_RIGHT_PICKUP_TRAJECTORY =
        "RedTrajectories/G-to-Right-Pickup";
    // Reef to Left Pickup
    public static final String RED_H_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/H-to-Left-Pickup";
    public static final String RED_I_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/I-to-Left-Pickup";
    public static final String RED_J_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/J-to-Left-Pickup";
    public static final String RED_K_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/K-to-Left-Pickup";
    public static final String RED_L_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/L-to-Left-Pickup";
    public static final String RED_A_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/A-to-Left-Pickup";
    public static final String RED_B_TO_LEFT_PICKUP_TRAJECTORY = "RedTrajectories/B-to-Left-Pickup";
    // Right Pickup to Reef
    public static final String RED_RIGHT_PICKUP_TO_A_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-A";
    public static final String RED_RIGHT_PICKUP_TO_B_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-B";
    public static final String RED_RIGHT_PICKUP_TO_C_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-C";
    public static final String RED_RIGHT_PICKUP_TO_D_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-D";
    public static final String RED_RIGHT_PICKUP_TO_E_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-E";
    public static final String RED_RIGHT_PICKUP_TO_F_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-F";
    public static final String RED_RIGHT_PICKUP_TO_G_TRAJECTORY =
        "RedTrajectories/Right-Pickup-to-G";
    // Left Pickup to Reef
    public static final String RED_LEFT_PICKUP_TO_H_TRAJECTORY = "RedTrajectories/Left-Pickup-to-H";
    public static final String RED_LEFT_PICKUP_TO_I_TRAJECTORY = "RedTrajectories/Left-Pickup-to-I";
    public static final String RED_LEFT_PICKUP_TO_J_TRAJECTORY = "RedTrajectories/Left-Pickup-to-J";
    public static final String RED_LEFT_PICKUP_TO_K_TRAJECTORY = "RedTrajectories/Left-Pickup-to-K";
    public static final String RED_LEFT_PICKUP_TO_L_TRAJECTORY = "RedTrajectories/Left-Pickup-to-L";
    public static final String RED_LEFT_PICKUP_TO_A_TRAJECTORY = "RedTrajectories/Left-Pickup-to-A";
    public static final String RED_LEFT_PICKUP_TO_B_TRAJECTORY = "RedTrajectories/Left-Pickup-to-B";
  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final double DEADBAND_VALUE = 0.05;
  }
}

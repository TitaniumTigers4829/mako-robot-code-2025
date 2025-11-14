package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.autodrive.AutoAlignPose;
import frc.robot.commands.autodrive.AutoAlignReef;
import frc.robot.commands.autodrive.RepulsorReef;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.FollowSwerveSampleCommand;
import frc.robot.commands.elevator.ScoreL4;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.funnel.SetFunnelAngle;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.ReefLocations;
import frc.robot.extras.util.ReefLocations.ReefBranch;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem.IntakeState;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnelPivot.FunnelConstants;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** This class is where all the auto routines are created. It also contains the auto chooser */
public class Autos {
  private final LoggedDashboardChooser<String> chooser;
  private final AutoFactory autoFactory;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private SwerveDrive swerveDrive;
  private VisionSubsystem visionSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private final String NONE_NAME = "Do Nothing";

  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();

  private final Pose2d sourceRight = new Pose2d(1.17, 1.23, Rotation2d.fromDegrees(54));

  Pose2d sourceRightFlipped =
      AllianceFlipper.isRed()
          ? sourceRight.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.kPi)
          : sourceRight;
  private final Pose2d sourceLeft =
      new Pose2d(
          sourceRightFlipped.getX(),
          FieldConstants.FIELD_WIDTH_METERS - sourceRightFlipped.getY(),
          sourceRightFlipped.getRotation().unaryMinus());

  @FunctionalInterface
  private interface ReefRepulsorCommand {
    Command goTo(ReefBranch branch);
  }

  @FunctionalInterface
  private interface SourceRepulsorCommand {
    Command goTo(Source source);
  }

  private final ReefRepulsorCommand reefPathfinding;
  private final SourceRepulsorCommand sourcePathfinding;

  private String selectedCommandName = NONE_NAME;
  private Command selectedCommand = Commands.none();
  private boolean selectedOnRed = false;

  public Autos(
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      FunnelSubsystem funnelSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);
    // this sets up the auto factory
    this.autoFactory =
        new AutoFactory(
            swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to the
            (SwerveSample sample) -> {
              FollowSwerveSampleCommand followSwerveSampleCommand =
                  new FollowSwerveSampleCommand(this.swerveDrive, this.visionSubsystem, sample);
              followSwerveSampleCommand.execute();
              Logger.recordOutput("Trajectory/sample", sample.getPose());
            }, // A function that follows a choreo trajectory
            false, // If alliance flipping should be enabled
            this.swerveDrive); // The drive subsystem

    reefPathfinding =
        branch ->
            new AutoAlignPose(
                    swerveDrive,
                    visionSubsystem,
                    ReefLocations.getScoringLocation(branch),
                    this::alignCallback)
                .withName("Reef Align " + branch.name());
    sourcePathfinding =
        source -> {
          Pose2d pose = source == Source.L ? sourceLeft : sourceRight;
          if (AllianceFlipper.isRed()) {
            pose = pose.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.kPi);
          }
          return new AutoAlignPose(swerveDrive, visionSubsystem, pose, this::alignCallback)
              .withName("Source Align " + source.name());
        };
    addRoutine("Blue Left Two Coral", () -> blueLeftTwoCoralAuto());

    addRoutine("Blue Right Two Coral", () -> blueRightTwoCoralAuto());

    addRoutine("Red Left Two Coral", () -> redLeftTwoCoralAuto());

    addRoutine("Red Right Two Coral Auto", () -> redRightTwoCoralAuto());

    addRoutine("middle auto", () -> simpleRepulsorAuto());

    addRoutine("x two meter", () -> xOneMeterAuto());

    addRoutine("y one meter", () -> yOneMeterAuto());

    addRoutine("y one meter and rotation", () -> yOneMeterAndRotationAuto());

    addRoutine(
        "fancy things",
        () ->
            createRoutine(
                autoFactory,
                swerveDrive,
                visionSubsystem,
                elevatorSubsystem,
                coralIntakeSubsystem,
                Source.L,
                ReefBranch.J,
                ReefBranch.K));

    addRoutine("dummy", () -> dumbShit());

    addRoutine("dummy again", () -> dumbShitRIght());
  }

  public AutoRoutine xOneMeterAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.X_ONE_METER_AUTO);
    AutoTrajectory xOneMeterTrajectory = routine.trajectory(AutoConstants.X_ONE_METER_TRAJECTORY);
    Logger.recordOutput("two meter auto", xOneMeterTrajectory.getRawTrajectory().getPoses());
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.X_ONE_METER_TRAJECTORY),
                xOneMeterTrajectory.cmd()));

    return routine;
  }

  public AutoRoutine yOneMeterAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.Y_ONE_METER_AUTO);
    AutoTrajectory yOneMeterTrajectory = routine.trajectory(AutoConstants.Y_ONE_METER_TRAJECTORY);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.Y_ONE_METER_TRAJECTORY),
                yOneMeterTrajectory.cmd()));

    return routine;
  }

  public AutoRoutine yOneMeterAndRotationAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Rotation auto");
    AutoTrajectory yOneMeterAndRotationAuto = routine.trajectory("RedTrajectories/rotatepath");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry("RedTrajectories/rotatepath"),
                yOneMeterAndRotationAuto.cmd()));

    return routine;
  }

  // Blue Auto Routines
  public AutoRoutine simpleRepulsorAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.SIMPLE_REPULSOR_AUTO);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())),
                new DriveCommand(
                        swerveDrive,
                        visionSubsystem,
                        () -> -0.13,
                        () -> 0.0,
                        () -> 0,
                        () -> false,
                        () -> false,
                        this::alignCallback)
                    .withTimeout(2.4829)
                    .andThen(
                        new SetFunnelAngle(funnelSubsystem, FunnelConstants.ANGLE_INTAKE)
                            .withTimeout(1.24829)),
                new InstantCommand(
                    () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())),
                new AutoAlignReef(swerveDrive, visionSubsystem, false, this::alignCallback)
                    .withTimeout(4),
                new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)));
    return routine;
  }

  public AutoRoutine dumbShit() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.SIMPLE_REPULSOR_AUTO);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())),
                new DriveCommand(
                        swerveDrive,
                        visionSubsystem,
                        () -> -0.24829,
                        () -> 0.0,
                        () -> 0,
                        () -> false,
                        () -> false,
                        this::alignCallback)
                    .withTimeout(1.94829)
                    .alongWith(
                        new SetFunnelAngle(funnelSubsystem, FunnelConstants.ANGLE_INTAKE)
                            .withTimeout(1.248294829)),
                new AutoAlignReef(swerveDrive, visionSubsystem, false, this::alignCallback)
                    .withTimeout(4),
                new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)
                    .andThen(
                        elevatorSubsystem
                            .setElevationPosition(ElevatorSetpoints.FEEDER.getPosition())
                            .alongWith(
                                new AutoAlignPose(
                                    swerveDrive, visionSubsystem, sourceLeft, this::alignCallback))
                            .raceWith(
                                Commands.sequence(
                                        // elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
                                        new InstantCommand(
                                            () ->
                                                coralIntakeSubsystem.setIntakeState(
                                                    IntakeState.IDLE)),
                                        Commands.run(
                                            () -> coralIntakeSubsystem.intakeCoral(),
                                            coralIntakeSubsystem))
                                    .until(() -> coralIntakeSubsystem.isIntakeComplete()))
                            .andThen(
                                new DriveCommand(
                                        swerveDrive,
                                        visionSubsystem,
                                        () -> -0.4829 + .24 + 0.04829,
                                        () -> -0.075,
                                        () -> -0.04829 + 0.035,
                                        () -> false,
                                        () -> false,
                                        this::alignCallback)
                                    .withTimeout(2.04829),
                                new WaitCommand(0.2),
                                new AutoAlignReef(
                                        swerveDrive, visionSubsystem, false, this::alignCallback)
                                    .withTimeout(4),
                                new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)))));
    return routine;
  }

  public AutoRoutine dumbShitRIght() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.SIMPLE_REPULSOR_AUTO);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(
                    () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())),
                new DriveCommand(
                        swerveDrive,
                        visionSubsystem,
                        () -> -0.24829,
                        () -> 0.0,
                        () -> 0,
                        () -> false,
                        () -> false,
                        this::alignCallback)
                    .withTimeout(1.94829)
                    .alongWith(
                        new SetFunnelAngle(funnelSubsystem, FunnelConstants.ANGLE_INTAKE)
                            .withTimeout(1.248294829)),
                new AutoAlignReef(swerveDrive, visionSubsystem, true, this::alignCallback)
                    .withTimeout(4),
                new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)
                    .andThen(
                        new WaitCommand(0.554829)
                            .andThen(
                                elevatorSubsystem.setElevationPosition(
                                    ElevatorSetpoints.FEEDER.getPosition()))
                            .alongWith(
                                new AutoAlignPose(
                                    swerveDrive,
                                    visionSubsystem,
                                    sourceRightFlipped,
                                    this::alignCallback))
                            .raceWith(
                                Commands.sequence(
                                        // elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
                                        new InstantCommand(
                                            () ->
                                                coralIntakeSubsystem.setIntakeState(
                                                    IntakeState.IDLE)),
                                        Commands.run(
                                            () -> coralIntakeSubsystem.intakeCoral(),
                                            coralIntakeSubsystem))
                                    .until(() -> coralIntakeSubsystem.isIntakeComplete()))
                            .andThen(
                                new DriveCommand(
                                        swerveDrive,
                                        visionSubsystem,
                                        () -> -0.4829 + .24 + 0.04829,
                                        () -> 0.075,
                                        () -> 0.04829 - 0.035,
                                        () -> false,
                                        () -> false,
                                        this::alignCallback)
                                    .withTimeout(1.94829),
                                new DriveCommand(
                                        swerveDrive,
                                        visionSubsystem,
                                        () -> 0.0,
                                        () -> 0.0,
                                        () -> 0.0,
                                        () -> false,
                                        () -> false,
                                        this::alignCallback)
                                    .withTimeout(0.35),
                                new AutoAlignReef(
                                        swerveDrive, visionSubsystem, false, this::alignCallback)
                                    .withTimeout(4),
                                new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)))));
    return routine;
  }

  private void alignCallback(boolean isAligned) {}

  public AutoRoutine blueLeftTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_LEFT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.BLUE_LEFT_PICKUP_TO_L_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_LEFT_START_TO_J_TRAJECTORY),
                // funnelSubsystem.dropFunnel(),
                // elevatorSubsystem.manualElevator(-0.2).withTimeout(0.2),
                // Commands.runOnce(() -> elevatorSubsystem.resetPosition(0.0), elevatorSubsystem),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(0.5)
                .andThen(
                    new InstantCommand(
                        () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())))
                .andThen(
                    new AutoAlignReef(swerveDrive, visionSubsystem, false, this::alignCallback))
                .withTimeout(4.0));
    // .andThen(new WaitCommand(0.1))
    // .andThen(
    //     new AutoAlignReef(swerveDrive, visionSubsystem, false, this::alignCallback))
    // .withTimeout(0.5)
    // .andThen(new WaitCommand(2)));

    //             //             .andThen(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem)));
    //             .andThen(jToPickupTrajectory.cmd())
    //             //         .alongWith(
    //             //             elevatorSubsystem.setElevationPosition(
    //             //                 ElevatorSetpoints.FEEDER.getPosition())))
    //             // .andThen(
    //             //     new InstantCommand(
    //             //         () ->
    // swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())))
    //             .andThen(
    //                 new AutoAlignPose(
    //                     swerveDrive, visionSubsystem, sourceLeft, this::alignCallback)));
    // jToPickupTrajectory
    //     .done()
    //     .onTrue(
    //         //         Commands.sequence(
    //         //
    //         // elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
    //         //                 new InstantCommand(() ->
    //         // coralIntakeSubsystem.setIntakeState(IntakeState.IDLE)),
    //         //                 Commands.runEnd(
    //         //                     () -> coralIntakeSubsystem.intakeCoral(),
    //         //                     () ->
    // coralIntakeSubsystem.setIntakeState(IntakeState.STOPPED),
    //         //                     coralIntakeSubsystem))
    //         // .andThen(
    //         pickupToLTrajectory
    //             .cmd()
    //             .andThen(
    //                 new AutoAlignReef(swerveDrive, visionSubsystem, true, this::alignCallback)));
    // );
    // pickupToLTrajectory
    //     .done()
    //     .onTrue(
    //         new WaitCommand(0.5)
    //             .andThen(
    //                 new RepulsorReef(swerveDrive, visionSubsystem, false)
    //                     .withTimeout(2.0)
    //                     .andThen(
    //                         new SetElevatorPosition(
    //                             swerveDrive, elevatorSubsystem,
    // ElevatorSetpoints.L4.getPosition()))
    //                     .withTimeout(2.0)
    //                     .andThen(
    //                         Commands.runEnd(
    //                                 () ->
    //                                     coralIntakeSubsystem.setIntakeVelocity(
    //                                         CoralIntakeConstants.EJECT_SPEED),
    //                                 () ->
    //                                     coralIntakeSubsystem.setIntakeVelocity(
    //                                         CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
    //                                 coralIntakeSubsystem)
    //                             .withTimeout(1.0))));
    //                     .alongWith(
    //                         new SetElevatorPosition(
    //                             swerveDrive, elevatorSubsystem,
    // ElevatorSetpoints.L4.getPosition()))
    //                     .withTimeout(4.0)
    //                     .andThen(
    //                         Commands.runEnd(
    //                                 () ->
    //                                     coralIntakeSubsystem.setIntakeVelocity(
    //                                         CoralIntakeConstants.EJECT_SPEED),
    //                                 () ->
    //                                     coralIntakeSubsystem.setIntakeVelocity(
    //                                         CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
    //                                 coralIntakeSubsystem)
    //                             .withTimeout(1.0))));

    // Logger.recordOutput("Trajectories/starttoj",
    // startToJTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    //     "Trajectories/jtopickup", jToPickupTrajectory.getRawTrajectory().getPoses());
    // Logger.recordOutput(
    // "Trajectories/pickuptoL", pickupToLTrajectory.getRawTrajectory().getPoses());
    // startToJTrajectory.done().onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    // startToJTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToLTrajectory.cmd());

    // rStartToETrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(eToPickupTrajectory.cmd());
    // eToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToDTrajectory.cmd());

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(hasCoral))
    //     .and(routine.observe(rightReefInRange))
    //     .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .anyActive(eToPickupTrajectory)
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    return routine;
  }

  public AutoRoutine blueRightTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_RIGHT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.BLUE_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.BLUE_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.BLUE_RIGHT_START_TO_E_TRAJECTORY),
                // funnelSubsystem.dropFunnel(),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(0.5)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, false)
                        .alongWith(
                            new SetElevatorPosition(
                                elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(4.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0)))
                .andThen(
                    eToPickupTrajectory
                        .cmd()
                        .alongWith(
                            elevatorSubsystem.setElevationPosition(
                                ElevatorSetpoints.FEEDER.getPosition()))));

    eToPickupTrajectory
        .done()
        .onTrue(
            Commands.sequence(
                    elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
                    new InstantCommand(() -> coralIntakeSubsystem.setIntakeState(IntakeState.IDLE)),
                    Commands.runEnd(
                        () -> coralIntakeSubsystem.intakeCoral(),
                        () -> coralIntakeSubsystem.setIntakeState(IntakeState.STOPPED),
                        coralIntakeSubsystem))
                .andThen(pickupToLTrajectory.cmd()));

    pickupToLTrajectory
        .done()
        .onTrue(
            new RepulsorReef(swerveDrive, visionSubsystem, false)
                .withTimeout(2.0)
                .andThen(
                    new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                .withTimeout(2.0)
                .andThen(
                    Commands.runEnd(
                            () ->
                                coralIntakeSubsystem.setIntakeVelocity(
                                    CoralIntakeConstants.EJECT_SPEED),
                            () ->
                                coralIntakeSubsystem.setIntakeVelocity(
                                    CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                            coralIntakeSubsystem)
                        .withTimeout(1.0)));
    return routine;
  }

  public AutoRoutine redLeftTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_LEFT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory lStartToJTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY);
    AutoTrajectory jToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_J_TO_LEFT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.RED_LEFT_PICKUP_TO_H_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_LEFT_START_TO_J_TRAJECTORY),
                lStartToJTrajectory.cmd()));
    lStartToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(.5)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, false)
                        .withTimeout(2.0)
                        .andThen(
                            new SetElevatorPosition(
                                elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(2.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0)))
                .andThen(
                    jToPickupTrajectory
                        .cmd()
                        .alongWith(
                            elevatorSubsystem.setElevationPosition(
                                ElevatorSetpoints.FEEDER.getPosition()))));

    jToPickupTrajectory
        .done()
        .onTrue(
            Commands.sequence(
                    elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
                    new InstantCommand(() -> coralIntakeSubsystem.setIntakeState(IntakeState.IDLE)),
                    Commands.runEnd(
                        () -> coralIntakeSubsystem.intakeCoral(),
                        () -> coralIntakeSubsystem.setIntakeState(IntakeState.STOPPED),
                        coralIntakeSubsystem))
                .andThen(pickupToLTrajectory.cmd()));

    pickupToLTrajectory
        .done()
        .onTrue(
            new RepulsorReef(swerveDrive, visionSubsystem, false)
                .withTimeout(2.0)
                .andThen(
                    new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                .withTimeout(2.0)
                .andThen(
                    Commands.runEnd(
                            () ->
                                coralIntakeSubsystem.setIntakeVelocity(
                                    CoralIntakeConstants.EJECT_SPEED),
                            () ->
                                coralIntakeSubsystem.setIntakeVelocity(
                                    CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                            coralIntakeSubsystem)
                        .withTimeout(1.0)));
    return routine;
  }

  public AutoRoutine redRightTwoCoralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.RED_RIGHT_TWO_CORAL_AUTO_ROUTINE);

    AutoTrajectory startToJTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY);
    AutoTrajectory eToPickupTrajectory =
        routine.trajectory(AutoConstants.RED_E_TO_RIGHT_PICKUP_TRAJECTORY);
    AutoTrajectory pickupToLTrajectory =
        routine.trajectory(AutoConstants.RED_RIGHT_PICKUP_TO_D_TRAJECTORY);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                autoFactory.resetOdometry(AutoConstants.RED_RIGHT_START_TO_E_TRAJECTORY),
                // funnelSubsystem.dropFunnel(),
                startToJTrajectory.cmd()));
    startToJTrajectory
        .done()
        .onTrue(
            new WaitCommand(2.0)
                .andThen(
                    new RepulsorReef(swerveDrive, visionSubsystem, false)
                        .alongWith(
                            new SetElevatorPosition(
                                elevatorSubsystem, ElevatorSetpoints.L4.getPosition()))
                        .withTimeout(4.0)
                        .andThen(
                            Commands.runEnd(
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.EJECT_SPEED),
                                    () ->
                                        coralIntakeSubsystem.setIntakeVelocity(
                                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                                    coralIntakeSubsystem)
                                .withTimeout(1.0))));

    // Logger.recordOutput(
    // "Trajectories/pickuptoL", pickupToLTrajectory.getRawTrajectory().getPoses());
    // startToJTrajectory.done().onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory.done().onTrue(pickupToLTrajectory.cmd());
    // startToJTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(jToPickupTrajectory.cmd());
    // jToPickupTrajectory
    //     .recentlyDone()
    //     .and(routine.observe(hasCoral))
    //     .onTrue(pickupToLTrajectory.cmd());

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(leftReefInRange).negate())
    //     .onTrue(
    //         Commands.sequence(
    //             new RepulsorReef(swerveDrive, visionSubsystem, true),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Repulsor Auto Trigger", true))));

    // routine
    //     .anyDone(startToJTrajectory, pickupToLTrajectory)
    //     .and(routine.observe(hasCoral))
    //     .and(routine.observe(rightReefInRange))
    //     .onTrue(new ScoreL4(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .anyActive(jToPickupTrajectory)
    //     .and(routine.observe(hasNoCoral))
    //     .onTrue(new IntakeCoral(elevatorSubsystem, coralIntakeSubsystem));

    // routine
    //     .observe(elevatorUpZone)
    //     .and(routine.observe(hasCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition()),
    //             new RunCommand(() -> SmartDashboard.putBoolean("Elevator Up Auto Trigger",
    // true))));
    // routine
    //     .observe(elevatorUpZone.negate())
    //     .or(routine.observe(hasNoCoral))
    //     .onTrue(
    //         Commands.sequence(
    //             new SetElevatorPosition(elevatorSubsystem,
    // ElevatorSetpoints.FEEDER.getPosition()),
    //             new RunCommand(
    //                 () -> SmartDashboard.putBoolean("Elevator Up Auto Trigger", false))));
    return routine;
  }

  private AutoRoutine createRoutine(
      AutoFactory factory,
      SwerveDrive swerve,
      VisionSubsystem visionSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      Source source,
      ReefBranch initialBranch,
      ReefBranch... cyclingBranches) {
    AutoRoutine routine = factory.newRoutine("Autogenerated Routine");

    AutoTrajectory[] reefToSource = new AutoTrajectory[cyclingBranches.length];
    AutoTrajectory[] sourceToReef = new AutoTrajectory[cyclingBranches.length];
    reefToSource[0] = getTrajectory(routine, initialBranch, source);
    sourceToReef[0] = getTrajectory(routine, source, cyclingBranches[0]);
    for (int i = 1; i < cyclingBranches.length; i++) {
      reefToSource[i] = getTrajectory(routine, cyclingBranches[i - 1], source);
      sourceToReef[i] = getTrajectory(routine, source, cyclingBranches[i]);
    }
    var nextCycleSpwnCmd = new Command[sourceToReef.length];
    for (int i = 0; i < sourceToReef.length - 1; i++) {
      nextCycleSpwnCmd[i] = reefToSource[i + 1].spawnCmd();
    }
    nextCycleSpwnCmd[sourceToReef.length - 1] =
        new ScheduleCommand(
            getTrajectory(routine, cyclingBranches[cyclingBranches.length - 1], source)
                .cmd()
                .andThen(sourcePathfinding.goTo(source)));

    routine
        .active()
        .onTrue(
            reefPathfinding
                .goTo(initialBranch)
                // Commands.run(
                //         () ->
                // elevatorSubsystem.setElevatorPosition(ElevatorSetpoints.L4.getPosition()))
                //     .onlyIf(() -> swerveDrive.isReefInRange())
                //     .deadlineFor(reefPathfinding.goTo(initialBranch))
                .withName("ScoreAt" + initialBranch.name())
                .andThen(reefToSource[0].spawnCmd())
                // .andThen(sourcePathfinding.goTo(source))
                .withName("StartTo" + initialBranch.name()));

    for (int i = 0; i < cyclingBranches.length; i++) {
      reefToSource[i]
          .done()
          .onTrue(
              //   waitUntil(() -> coralIntakeSubsystem.hasControl())
              // .withTimeout(.5) // ONLY RUN IN SIM
              //   .deadlineFor(
              sourcePathfinding
                  .goTo(source)
                  // )
                  .andThen(sourceToReef[i].spawnCmd())
                  .withName("Source" + source.name()));

      sourceToReef[i]
          .atTimeBeforeEnd(.5)
          .onTrue(
              //   Commands.run(
              //           () ->
              //
              // elevatorSubsystem.setElevatorPosition(ElevatorSetpoints.L4.getPosition()))
              //       .onlyIf(() -> swerveDrive.isReefInRange())
              //   .deadlineFor(
              Commands.waitUntil(sourceToReef[i].done())
                  .andThen(reefPathfinding.goTo(cyclingBranches[i]).asProxy())
                  //   )
                  .andThen(nextCycleSpwnCmd[i])
                  .withName("ScoreAt" + cyclingBranches[i].name()));
    }

    return routine;
  }

  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);
  private final Alert loadedAutoAlert = new Alert("", Alert.AlertType.kInfo);

  public void update() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      String selected = chooser.get();
      if (selected.equals(selectedCommandName) && selectedOnRed == AllianceFlipper.isRed()) {
        return;
      }
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get().withName(selectedCommandName);
      selectedOnRed = AllianceFlipper.isRed();
      loadedAutoAlert.setText("Loaded Auto: " + selectedCommandName);
      loadedAutoAlert.set(true);
    }
  }

  public void clear() {
    selectedCommandName = NONE_NAME;
    selectedCommand = Commands.none();
    selectedOnRed = false;
  }

  public Command getSelectedCommand() {
    return selectedCommand;
  }

  private void addRoutine(String name, Supplier<AutoRoutine> generator) {
    chooser.addOption(name, name);
    routines.put(name, () -> generator.get().cmd());
  }

  private enum Source {
    L,
    R
  }

  private AutoTrajectory getTrajectory(AutoRoutine routine, ReefBranch reefBranch, Source source) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 0);
  }

  private AutoTrajectory getTrajectory(AutoRoutine routine, Source source, ReefBranch reefBranch) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 1);
  }
}

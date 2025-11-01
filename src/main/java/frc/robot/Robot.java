package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.autodrive.AutoAlignReef;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.sim.SimWorld;
import frc.robot.subsystems.climbPivot.ClimbPivotInterface;
import frc.robot.subsystems.climbPivot.ClimbPivotSubsystem;
import frc.robot.subsystems.climbPivot.PhysicalClimbPivot;
import frc.robot.subsystems.climbPivot.SimulatedClimbPivot;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeInterface;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem.IntakeState;
import frc.robot.subsystems.coralIntake.PhysicalCoralIntake;
import frc.robot.subsystems.coralIntake.SimulatedCoralntake;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorInterface;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.PhysicalElevator;
import frc.robot.subsystems.elevator.SimulatedElevator;
import frc.robot.subsystems.funnelPivot.FunnelPivotInterface;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;
import frc.robot.subsystems.funnelPivot.PhysicalFunnelPivot;
import frc.robot.subsystems.funnelPivot.SimulatedFunnelPivot;
import frc.robot.subsystems.leds.LEDConstants.LEDProcess;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.gyro.PhysicalGyroNavX;
import frc.robot.subsystems.swerve.gyro.PhysicalGyroPigeon;
import frc.robot.subsystems.swerve.gyro.SimulatedGyro;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.swerve.module.PhysicalModule;
import frc.robot.subsystems.swerve.module.SimulatedModule;
import frc.robot.subsystems.vision.PhysicalVision;
import frc.robot.subsystems.vision.SimulatedVision;
import frc.robot.subsystems.vision.VisionInterface;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private VisionSubsystem visionSubsystem;
  private SwerveDrive swerveDrive;
  private ElevatorSubsystem elevatorSubsystem;
  private CoralIntakeSubsystem coralIntakeSubsystem;
  private FunnelSubsystem funnelSubsystem;
  private ClimbPivotSubsystem climbPivotSubsystem;
  private LEDSubsystem ledSubsystem;

  private SimWorld simWorld;
  private Autos autos;
  private Command autoCommand;

  public Robot() {
    checkGit();
    setupLogging();
    setupSubsystems();
    setupAuto();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority and real time to improve loop timing
    Threads.setCurrentThreadPriority(true, HardwareConstants.HIGH_THREAD_PRIORITY);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority without real time
    Threads.setCurrentThreadPriority(false, HardwareConstants.LOW_THREAD_PRIORITY);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autos.update();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoCommand = autos.getSelectedCommand();
    autoCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
    autos.clear();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDriverController();
    configureOperatorController();
  }

  /**
   * Callback for the auto align command, will rumble the controllers when aligned
   *
   * @param isAligned whether the robot is aligned
   */
  private void alignCallback(boolean isAligned) {
    if (isAligned) {
      driverController.setRumble(RumbleType.kBothRumble, 0.5);
      operatorController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      driverController.setRumble(RumbleType.kBothRumble, 0.0);
      operatorController.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  /** Configures the driver controller buttons and axes to control the robot */
  private void configureDriverController() {
    // Driver Left Stick
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[1],
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[0]
        };

    Command driveCommand =
        new DriveCommand(
            swerveDrive,
            visionSubsystem,
            // Translation in the X direction
            driverLeftStick[0],
            // Translation in the Y direction
            driverLeftStick[1],
            // Rotation
            () -> JoystickUtil.modifyAxis(driverController::getRightX, 3),
            // Robot relative
            () -> !driverController.rightBumper().getAsBoolean(),
            // Rotation speed
            () -> driverController.rightStick().getAsBoolean(),
            this::alignCallback);

    // Sets the default command for the swerve drive to the drive command
    swerveDrive.setDefaultCommand(driveCommand);

    // Auto align command for the right reef
    driverController
        .rightTrigger()
        .whileTrue(new AutoAlignReef(swerveDrive, visionSubsystem, false, this::alignCallback));

    // Auto align command for the left reef
    driverController
        .leftTrigger()
        .whileTrue(
            new InstantCommand(
                    () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose()))
                .andThen(
                    new AutoAlignReef(swerveDrive, visionSubsystem, true, this::alignCallback)));

    // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverController
        .povRight()
        .onTrue(
            new InstantCommand(
                () ->
                    swerveDrive.resetEstimatedPose(
                        new Pose2d(
                            swerveDrive.getEstimatedPose().getX(),
                            swerveDrive.getEstimatedPose().getY(),
                            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())))));

    // Reset robot odometry based on the most recent vision pose measurement from april tags
    // This should be pressed when looking at an april tag
    driverController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())));
  }

  /** Configures the operator controller buttons and axes to control the robot */
  private void configureOperatorController() {
    // ELEVATOR COMMANDS
    // Toggle the elevator limits
    operatorController.povLeft().onTrue(new InstantCommand(elevatorSubsystem::toggleLimits));

    // Manual elevator control
    operatorController
        .rightBumper()
        .whileTrue(
            elevatorSubsystem
                .manualElevator(() -> -operatorController.getLeftY())
                .onlyIf(
                    () ->
                        (coralIntakeSubsystem.isIntakeComplete()
                            || coralIntakeSubsystem.isIntakeIdle())));

    // Zero the elevator position
    operatorController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> elevatorSubsystem.resetPosition(0.0), elevatorSubsystem));
    // Sets the elevator to L1 position
    operatorController
        .a()
        .whileTrue(
            new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L1.getPosition())
                .onlyIf(
                    () ->
                        (coralIntakeSubsystem.isIntakeComplete()
                            || coralIntakeSubsystem.isIntakeIdle())));
    // Sets the elevator to L2 position
    operatorController
        .x()
        .whileTrue(
            new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L2.getPosition())
                .onlyIf(
                    () ->
                        (coralIntakeSubsystem.isIntakeComplete()
                            || coralIntakeSubsystem.isIntakeIdle())));
    // Sets the elevator to L3 position
    operatorController
        .b()
        .whileTrue(
            new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L3.getPosition())
                .onlyIf(
                    () ->
                        (coralIntakeSubsystem.isIntakeComplete()
                            || coralIntakeSubsystem.isIntakeIdle())));
    // Sets the elevator to L4 position
    operatorController
        .y()
        .whileTrue(
            new SetElevatorPosition(elevatorSubsystem, ElevatorSetpoints.L4.getPosition())
                .onlyIf(
                    () ->
                        (coralIntakeSubsystem.isIntakeComplete()
                            || coralIntakeSubsystem.isIntakeIdle())));
    // INTAKE COMMANDS
    // Reverse intake command
    operatorController
        .povRight()
        .whileTrue(
            Commands.runEnd(
                    () ->
                        coralIntakeSubsystem.setIntakeVelocity(
                            CoralIntakeConstants.REVERSE_INTAKE_SPEED + -1800),
                    () ->
                        coralIntakeSubsystem.setIntakeVelocity(
                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                    coralIntakeSubsystem)
                .andThen(
                    Commands.runOnce(
                        () -> coralIntakeSubsystem.setIntakeState(IntakeState.IDLE),
                        coralIntakeSubsystem)));

    // Eject command
    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.runEnd(
                    () -> coralIntakeSubsystem.setIntakeVelocity(CoralIntakeConstants.EJECT_SPEED),
                    () ->
                        coralIntakeSubsystem.setIntakeVelocity(
                            CoralIntakeConstants.NEUTRAL_INTAKE_SPEED),
                    coralIntakeSubsystem)
                .andThen(
                    Commands.runOnce(
                        () -> coralIntakeSubsystem.setIntakeState(IntakeState.IDLE),
                        coralIntakeSubsystem)));

    // Intake command
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                elevatorSubsystem.setElevationPosition(ElevatorSetpoints.FEEDER.getPosition()),
                new InstantCommand(() -> coralIntakeSubsystem.setIntakeState(IntakeState.IDLE)),
                Commands.runEnd(
                    () -> coralIntakeSubsystem.intakeCoral(),
                    () -> coralIntakeSubsystem.setIntakeState(IntakeState.STOPPED),
                    coralIntakeSubsystem)));

    // OTHER COMMANDS
    // Manual climb control
    operatorController
        .povUp()
        .whileTrue(
            climbPivotSubsystem.manualPivotClimb(
                () -> JoystickUtil.modifyAxis(() -> operatorController.getLeftY(), 2)));

    // Manual funnel control
    operatorController
        .povDown()
        .whileTrue(
            funnelSubsystem.manualFunnel(
                () -> JoystickUtil.modifyAxis(() -> operatorController.getLeftY(), 2)));
  }

  /** Checks the git status and records it to the log */
  private void checkGit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // This tells you if you have uncommitted changes in your project
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }

  /** Sets up the AdvantageKit logger */
  private void setupLogging() {
    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Gets data from network tables
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        {
          // Replaying a log, set up replay source
          setUseTiming(false); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  @Override
  public void robotInit() {}

  /** Sets up the subsystems based on the robot type */
  private void setupSubsystems() {
    switch (Constants.getRobot()) {
      case COMP_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyroPigeon(),
                new PhysicalModule(SwerveConstants.compModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision());
        this.elevatorSubsystem = new ElevatorSubsystem(new PhysicalElevator());
        this.funnelSubsystem = new FunnelSubsystem(new PhysicalFunnelPivot());
        this.climbPivotSubsystem = new ClimbPivotSubsystem(new PhysicalClimbPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new PhysicalCoralIntake());
        this.ledSubsystem = new LEDSubsystem();
        this.simWorld = null;
      }
      case DEV_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyroNavX(),
                new PhysicalModule(SwerveConstants.devModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.devModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.devModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.devModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision());
        this.elevatorSubsystem = new ElevatorSubsystem(new PhysicalElevator());
        this.funnelSubsystem = new FunnelSubsystem(new PhysicalFunnelPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new PhysicalCoralIntake());
        this.climbPivotSubsystem = new ClimbPivotSubsystem(new PhysicalClimbPivot());
        this.ledSubsystem = new LEDSubsystem();
        this.simWorld = null;
      }
      case SWERVE_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyroNavX(),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.aquilaModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision());
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorInterface() {});
        this.funnelSubsystem = new FunnelSubsystem(new PhysicalFunnelPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeInterface() {});
        this.climbPivotSubsystem = new ClimbPivotSubsystem(new PhysicalClimbPivot());
        this.simWorld = null;
      }

      case SIM_ROBOT -> {
        /* Sim robot, instantiate physics sim IO implementations */
        this.simWorld = new SimWorld();
        this.swerveDrive =
            new SwerveDrive(
                new SimulatedGyro(simWorld.robot().getDriveTrain().getGyro()),
                new SimulatedModule(0, simWorld.robot().getDriveTrain()),
                new SimulatedModule(1, simWorld.robot().getDriveTrain()),
                new SimulatedModule(2, simWorld.robot().getDriveTrain()),
                new SimulatedModule(3, simWorld.robot().getDriveTrain()));

        this.visionSubsystem =
            new VisionSubsystem(new SimulatedVision(() -> simWorld.aprilTagSim()));
        this.swerveDrive.resetEstimatedPose(new Pose2d(7, 4, new Rotation2d()));
        this.elevatorSubsystem = new ElevatorSubsystem(new SimulatedElevator());
        this.funnelSubsystem = new FunnelSubsystem(new SimulatedFunnelPivot());
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new SimulatedCoralntake());
        this.climbPivotSubsystem = new ClimbPivotSubsystem(new SimulatedClimbPivot());
        this.ledSubsystem = new LEDSubsystem();
        SmartDashboard.putBoolean("Coral", false);
      }

      default -> {
        this.visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        this.swerveDrive =
            new SwerveDrive(
                new GyroInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {},
                new ModuleInterface() {});
        this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorInterface() {});
        this.funnelSubsystem = new FunnelSubsystem(new FunnelPivotInterface() {});
        this.coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeInterface() {});
        this.climbPivotSubsystem = new ClimbPivotSubsystem(new ClimbPivotInterface() {});
        this.simWorld = null;
      }
    }
    ledSubsystem.setProcess(LEDProcess.DEFAULT);
  }

  /** Sets up the auto commands */
  private void setupAuto() {
    this.autos =
        new Autos(
            this.elevatorSubsystem,
            this.coralIntakeSubsystem,
            this.swerveDrive,
            this.visionSubsystem,
            this.funnelSubsystem);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (simWorld != null) {
      simWorld.update(() -> swerveDrive.getEstimatedPose());
    }
  }
}

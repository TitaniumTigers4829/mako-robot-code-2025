package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Robot;
import frc.robot.extras.logging.Tracer;
import frc.robot.extras.swerve.RepulsorFieldPlanner;
import frc.robot.extras.swerve.RepulsorFieldPlanner.RepulsorSample;
import frc.robot.extras.swerve.setpointGen.SwerveSetpoint;
import frc.robot.extras.swerve.setpointGen.SwerveSetpointGenerator;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.ReefLocations;
import frc.robot.extras.util.TimeUtil;
import frc.robot.sim.configs.SimSwerveModuleConfig.WheelCof;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  private final GyroInterface gyroIO;
  private final GyroInputsAutoLogged gyroInputs;
  private final SwerveModule[] swerveModules;
  private final PIDController xChoreoController =
      new PIDController(
          AutoConstants.CHOREO_AUTO_TRANSLATION_P,
          AutoConstants.CHOREO_AUTO_TRANSLATION_I,
          AutoConstants.CHOREO_AUTO_TRANSLATION_D);
  private final PIDController yChoreoController =
      new PIDController(
          AutoConstants.CHOREO_AUTO_TRANSLATION_P,
          AutoConstants.CHOREO_AUTO_TRANSLATION_I,
          AutoConstants.CHOREO_AUTO_TRANSLATION_D);
  private final ProfiledPIDController rotationChoreoController =
      new ProfiledPIDController(
          AutoConstants.CHOREO_AUTO_THETA_P,
          AutoConstants.CHOREO_AUTO_THETA_I,
          AutoConstants.CHOREO_AUTO_THETA_D,
          AutoConstants.AUTO_ALIGN_ROTATION_CONSTRAINTS);

  private Rotation2d rawGyroRotation;
  private final SwerveModulePosition[] lastModulePositions;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final RepulsorFieldPlanner repulsorFieldPlanner = new RepulsorFieldPlanner();

  private final PIDController xController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(8, 0, 0.0);

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          DriveConstants.MODULE_TRANSLATIONS,
          DCMotor.getFalcon500(1).withReduction(ModuleConstants.DRIVE_GEAR_RATIO),
          DCMotor.getFalcon500(1).withReduction(ModuleConstants.TURN_GEAR_RATIO),
          60,
          58,
          7,
          ModuleConstants.WHEEL_DIAMETER_METERS,
          WheelCof.BLACK_NITRILE.cof,
          0.0);
  private SwerveSetpoint setpoint = SwerveSetpoint.zeroed();

  private Optional<DriverStation.Alliance> alliance;

  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro Hardware Fault", Alert.AlertType.kError);

  public SwerveDrive(
      GyroInterface gyroIO,
      ModuleInterface frontLeftModuleIO,
      ModuleInterface frontRightModuleIO,
      ModuleInterface backLeftModuleIO,
      ModuleInterface backRightModuleIO) {
    this.gyroIO = gyroIO;
    this.gyroInputs = new GyroInputsAutoLogged();
    this.rawGyroRotation = new Rotation2d();

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(frontLeftModuleIO, "FrontLeft"),
          new SwerveModule(frontRightModuleIO, "FrontRight"),
          new SwerveModule(backLeftModuleIO, "BackLeft"),
          new SwerveModule(backRightModuleIO, "BackRight")
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(
                DriveConstants.X_POS_TRUST, DriveConstants.Y_POS_TRUST, DriveConstants.ANGLE_TRUST),
            VecBuilder.fill(
                VisionConstants.VISION_X_POS_TRUST,
                VisionConstants.VISION_Y_POS_TRUST,
                VisionConstants.VISION_ANGLE_TRUST));

    rotationChoreoController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    gyroDisconnectedAlert.set(false);
  }

  @Override
  public void periodic() {
    final double t0 = TimeUtil.getRealTimeSeconds();
    updateSwerveInputs();
    Logger.recordOutput(
        "SystemPerformance/OdometryFetchingTimeMS", (TimeUtil.getRealTimeSeconds() - t0) * 1000);
    // Runs the SwerveModules periodic methods
    modulesPeriodic();
  }

  /**
   * Drives the robot using the joysticks.
   *
   * @param xSpeed Speed of the robot in the x direction, positive being forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    ChassisSpeeds desiredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotationSpeed, getOdometryAllianceRelativeRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

    setpoint =
        setpointGenerator.generateSimpleSetpoint(
            setpoint, desiredSpeeds, HardwareConstants.LOOP_TIME_SECONDS);

    setModuleStates(setpoint.moduleStates());
    Logger.recordOutput("SwerveStates/DesiredStates", setpoint.moduleStates());
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    drive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        fieldRelative);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return setpoint.chassisSpeeds();
  }

  /**
   * Allows PID on the chassis rotation.
   *
   * @param speeds The ChassisSpeeds of the drive to set.
   * @param rotationControl The control on the drive rotatio /* Updates the pose estimator with the
   *     pose calculated from the april tags. How much it contributes to the pose estimation is set
   *     by setPoseEstimatorVisionConfidence.
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the pose from the april tags
   *     was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(
      Pose2d visionMeasurement, double currentTimeStampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  /**
   * Sets the standard deviations of model states, or how much the april tags contribute to the pose
   * estimation of the robot. Lower numbers equal higher confidence and vice versa.
   *
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(
      double xStandardDeviation, double yStandardDeviation, double thetaStandardDeviation) {
    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

  /**
   * Runs characterization on voltage
   *
   * @param volts current to set
   */
  public void runCharacterizationVoltage(double volts) {
    for (SwerveModule module : swerveModules) {
      module.setVoltage(Volts.of(-volts));
    }
  }

  /**
   * Runs characterization on current
   *
   * @param amps current to set
   */
  public void runCharacterizationCurrent(double amps) {
    for (SwerveModule module : swerveModules) {
      module.setCurrent(Amps.of(-amps));
    }
  }

  /**
   * @param omegaspeed Controls the rotation speed of the drivetrain for characterization.
   */
  public void runWheelRadiusCharacterization(double omegaspeed) {
    drive(0, 0, omegaspeed, false);
  }

  /**
   * Gets the wheel radiues characterization position
   *
   * @return returns the averaged wheel positions.
   */
  public double[] getWheelRadiusCharacterizationPosition() {
    double[] wheelPositions = new double[swerveModules.length];

    // Iterate over all the swerve modules, get their positions and add them to the array
    for (SwerveModule module : swerveModules) {
      wheelPositions[swerveModules.length - 1] = module.getDrivePositionRadians();
    }
    return wheelPositions;
  }

  /**
   * Gets the total characterization velocity of the modules.
   *
   * @return the summed characterization velocity of the modules
   */
  public double getCharacterizationVelocity() {
    double velocity = 0.0;
    for (SwerveModule module : swerveModules) {
      velocity += module.getCharacterizationVelocity();
    }
    return velocity;
  }

  /** Updates and logs the inputs for the odometry thread, gyro, and swerve modules. */
  private void updateSwerveInputs() {
    for (SwerveModule module : swerveModules) module.updateOdometryInputs();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Tracer.traceFunc("Gyro", () -> gyroIO.updateInputs(gyroInputs));
    gyroDisconnectedAlert.set(!gyroInputs.isConnected);
  }

  /**
   * Moves the robot to a sample(one point) of a swerve Trajectory provided by Choreo
   *
   * @param sample trajectory
   */
  public void followSwerveSample(SwerveSample sample) {
    // Use the summed forces in the drive method
    ChassisSpeeds chassisSpeeds;
    // if (fieldRelative) {
    //   chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //       totalForceX + moveX, totalForceY + moveY, moveTheta, getOdometryRotation2d());
    // } else {
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx,
            // + totalForcesX
            // + xChoreoController.calculate(getEstimatedPose().getX(), sample.x),
            sample.vy,
            // + yChoreoController.calculate(getEstimatedPose().getY(), sample.vy),
            sample.omega,
            getOdometryRotation2d());
    Logger.recordOutput("Trajectories/CurrentX", getEstimatedPose().getX());
    Logger.recordOutput("Trajectories/DesiredX", sample.x);
    Logger.recordOutput("Trajectories/vx", sample.vx);
    // }
    // double moveX = sample.vx;
    // // + xChoreoController.calculate(getEstimatedPose().getX(), sample.x);
    // double moveY = sample.vy;
    // // + yChoreoController.calculate(getEstimatedPose().getY(), sample.y);
    // double moveTheta = sample.omega;
    //     + rotationChoreoController.calculate(
    //         getOdometryRotation2d().getRadians(), sample.heading);
    chassisSpeeds = Robot.isSimulation() ? chassisSpeeds : chassisSpeeds.unaryMinus();
    drive(chassisSpeeds, false);
  }

  /** Runs the SwerveModules periodic methods */
  private void modulesPeriodic() {
    for (SwerveModule module : swerveModules) module.periodic();
  }

  /**
   * Returns if the robot speed is to zero when zeroed
   *
   * @return is robot moving along x
   * @return is robot moving along y
   * @return is robot rotating
   */
  public boolean getZeroedSpeeds(ChassisSpeeds speeds) {
    return speeds.vxMetersPerSecond == 0
        && speeds.vyMetersPerSecond == 0
        && speeds.omegaRadiansPerSecond == 0;
  }

  /**
   * Returns the heading of the robot in degrees from 0 to 360.
   *
   * @return Value is Counter-clockwise positive.
   */
  public double getHeading() {
    return gyroInputs.yawDegrees;
  }

  /**
   * Gets the rate of rotation of the robot.
   *
   * @return The current rate in degrees per second.
   */
  public double getGyroRate() {
    return gyroInputs.yawVelocityDegreesPerSecond;
  }

  /**
   * Gets the current roll of the gyro
   *
   * @return idk
   */
  public double getGyroRoll() {
    return gyroInputs.rollDegrees;
  }

  /**
   * Gets the current pitch of the gyro
   *
   * @return smthn
   */
  public double getGyroPitch() {
    return gyroInputs.pitchDegrees;
  }

  /**
   * Gets the rotation of the robot represented as a Rotation2d.
   *
   * @return a Rotation2d for the heading of the robot.
   */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Gets the rotation of the robot relative to the field from the driver's perspective. This means
   * that if the driver is on the red alliance and this method says the robot is facing 0 degrees,
   * the robot is actually facing 180 degrees and facing the blue alliance wall.
   *
   * @return a Rotation2d for the heading of the robot relative to the field from the driver's
   *     perspective.
   */
  public Rotation2d getGyroFieldRelativeRotation2d() {
    return Rotation2d.fromDegrees(getHeading() + getAllianceAngleOffset());
  }

  /**
   * Gets the angle offset of the robot based on the alliance color.
   *
   * @return 0 degrees if the robot is on the blue alliance, 180 if on the red alliance.
   */
  public double getAllianceAngleOffset() {
    alliance = DriverStation.getAlliance();
    // If theres a glitch in the FMS and for some reason we don't know if we're red or blue, just
    // assume we're blue
    // This should NEVER happen
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180.0 : 0.0;
  }

  /**
   * Resets the gyro heading to zero. When this is called, the robot's current heading will be
   * considered foward.
   */
  public void zeroHeading() {
    gyroIO.reset();
  }

  /**
   * Gets the estimated field-relative pose of the robot. Positive x being forward, positive y being
   * left. We estimate the robot's pose using the swerve modules, gyro, and vision.
   *
   * @return the estimated pose of the robot
   */
  @AutoLogOutput(key = "Odometry/Odometry")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the rotation of the robot from the odometry. This value is only influenced by the gyro as
   * we have set the standard deviations of the rotation from the vision measurements to a very high
   * number (this means we have very low confidence in the rotation from vision measurements). The
   * gyro only drifts a very small amount over time, so this value is very accurate.
   *
   * <p>For pretty much everything, use this method to get the rotation of the robot. This value can
   * differ from the gyro rotation for multiple reasons, but this is the value used by anything
   * autonomous.
   *
   * @return a Rotation2d for the heading of the robot.
   */
  public Rotation2d getOdometryRotation2d() {
    return getEstimatedPose().getRotation();
  }

  /**
   * Gets the Rotation2d for the heading of the robot relative to the field from the driver's
   * perspective. This method is needed so that the drive command and poseEstimator don't fight each
   * other. It uses odometry rotation.
   *
   * @return a Rotation2d for the heading of the robot relative to the field from the driver's
   *     perspective.
   */
  public Rotation2d getOdometryAllianceRelativeRotation2d() {
    return getEstimatedPose().getRotation().plus(Rotation2d.fromDegrees(getAllianceAngleOffset()));
  }

  /**
   * Sets the modules to the specified states. This is what actually tells the modules what to do,
   * so setting their rotation and speeds.
   *
   * @param desiredStates The desired states for the swerve modules. The order is: frontLeft,
   *     frontRight, backLeft, backRight (should be the same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setOptimizedDesiredState((desiredStates[i]));
    }
  }

  /** Sets the modules to form an X stance. */
  public void setXStance() {
    Rotation2d[] swerveHeadings = new Rotation2d[swerveModules.length];
    for (int i = 0; i < 4; i++) {
      swerveHeadings[i] = Rotation2d.fromDegrees(45);
    }
    DriveConstants.DRIVE_KINEMATICS.resetHeadings(swerveHeadings);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].stopModule();
    }
  }

  /**
   * Updates the pose estimator with the pose calculated from the swerve modules. This works because
   * if you know the circumference of the wheel and the angle of the wheel, you can calculate the
   * distance the wheel has traveled. This is done for all the wheels and then the pose estimator is
   * updated with this information.
   */
  public void addPoseEstimatorSwerveMeasurement() {
    final SwerveModulePosition[] modulePositions = getModulePositions(),
        moduleDeltas = getModulesDelta(modulePositions);

    // If the gyro is connected, use the gyro rotation. If not, use the calculated rotation from the
    // modules.
    if (gyroInputs.isConnected) {
      rawGyroRotation = getGyroRotation2d();
    } else {
      Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    poseEstimator.updateWithTime(TimeUtil.getLogTimeSeconds(), rawGyroRotation, modulePositions);
  }

  /**
   * Gets the change in the module positions between the current and last update.
   *
   * @param freshModulesPosition Latest module positions
   * @return The change of the module distances and angles since the last update.
   */
  private SwerveModulePosition[] getModulesDelta(SwerveModulePosition[] freshModulesPosition) {
    SwerveModulePosition[] deltas = new SwerveModulePosition[swerveModules.length];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      final double deltaDistanceMeters =
          freshModulesPosition[moduleIndex].distanceMeters
              - lastModulePositions[moduleIndex].distanceMeters;
      deltas[moduleIndex] =
          new SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex].angle);
      lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex];
    }
    return deltas;
  }

  /**
   * Gets the module states (speed and angle) for all the modules.
   *
   * @return The module states for all the modules in the order: frontLeft, frontRight, backLeft,
   *     backRight.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getMeasuredState();
    return states;
  }

  /**
   * Gets the module positions (distance and angle) for all the modules.
   *
   * @return The module positions for all the modules in the order: frontLeft, frontRight, backLeft,
   *     backRight.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < positions.length; i++) positions[i] = swerveModules[i].getPosition();
    return positions;
  }

  /**
   * Resets the estimated pose of the robot. The gyro does not need to be reset as the pose
   * estimator will take care of that.
   *
   * @param pose the new pose to set the robot to
   */
  public void resetEstimatedPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  @AutoLogOutput
  public boolean nearSource() {
    double maxX = 3.5;
    if (AllianceFlipper.isRed()) {
      return poseEstimator.getEstimatedPosition().getX()
          > (FieldConstants.FIELD_WIDTH_METERS - maxX);
    } else {
      return poseEstimator.getEstimatedPosition().getX() < maxX;
    }
  }

  public void sourceAlign(Supplier<Translation2d> translationalControlSupplier) {
    headingController.reset();
    double targetAngle = Units.degreesToRadians(54);
    if (AllianceFlipper.isRed()) {
      targetAngle = Math.PI - targetAngle;
    }
    if (poseEstimator.getEstimatedPosition().getY() > FieldConstants.FIELD_WIDTH_METERS / 2) {
      targetAngle *= -1;
    }

    var translationalControl = translationalControlSupplier.get();

    ChassisSpeeds commandedRobotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                translationalControl.getX() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                translationalControl.getY() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                headingController.calculate(
                    poseEstimator.getEstimatedPosition().getRotation().getRadians(), targetAngle)),
            getOdometryRotation2d());

    drive(commandedRobotSpeeds, false);
  }

  public void followRepulsorField(Pose2d goal) {
    followRepulsorField(goal, null);
  }

  public void followRepulsorField(Pose2d goal, Supplier<Translation2d> nudgeSupplier) {

    repulsorFieldPlanner.setGoal(goal.getTranslation());
    xController.reset();
    yController.reset();
    headingController.reset();
    Logger.recordOutput("Repulsor/Goal", goal);

    RepulsorSample sample =
        repulsorFieldPlanner.sampleField(
            poseEstimator.getEstimatedPosition().getTranslation(),
            DriveConstants.MAX_SPEED_METERS_PER_SECOND * .9,
            1.25);

    ChassisSpeeds feedforward = new ChassisSpeeds(sample.vx(), sample.vy(), 0);
    ChassisSpeeds feedback =
        new ChassisSpeeds(
            MathUtil.clamp(
                xController.calculate(
                    poseEstimator.getEstimatedPosition().getX(), sample.intermediateGoal().getX()),
                0,
                3),
            MathUtil.clamp(
                yController.calculate(
                    poseEstimator.getEstimatedPosition().getY(), sample.intermediateGoal().getY()),
                0,
                3),
            headingController.calculate(
                poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                goal.getRotation().getRadians()));

    var error = goal.minus(poseEstimator.getEstimatedPosition());
    Logger.recordOutput("Repulsor/Error", error);
    Logger.recordOutput("Repulsor/Feedforward", feedforward);
    Logger.recordOutput("Repulsor/Feedback", feedback);

    //                  Logger.recordOutput("Repulsor/Vector field",
    // repulsorFieldPlanner.getArrows());

    ChassisSpeeds outputFieldRelative = feedforward.plus(feedback);

    if (nudgeSupplier != null) {
      Translation2d nudge = nudgeSupplier.get();
      if (nudge.getNorm() > .1) {
        double nudgeScalar =
            Math.min(error.getTranslation().getNorm() / 3, 1)
                * Math.min(error.getTranslation().getNorm() / 3, 1)
                * DriveConstants.MAX_SPEED_METERS_PER_SECOND;

        if (AllianceFlipper.isRed()) {
          nudge = new Translation2d(-nudge.getX(), -nudge.getY());
        }
        nudgeScalar *=
            Math.abs(
                nudge
                    .getAngle()
                    .minus(
                        new Rotation2d(
                            outputFieldRelative.vxMetersPerSecond,
                            outputFieldRelative.vyMetersPerSecond))
                    .getSin());
        outputFieldRelative.vxMetersPerSecond += nudge.getX() * nudgeScalar;
        outputFieldRelative.vyMetersPerSecond += nudge.getY() * nudgeScalar;
      }
    }

    ChassisSpeeds outputRobotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            outputFieldRelative, poseEstimator.getEstimatedPosition().getRotation());

    outputRobotRelative =
        Robot.isSimulation() ? outputRobotRelative : outputRobotRelative.unaryMinus();

    drive(outputRobotRelative, false);
  }

  /**
   * Checks if the robot is within a certain distance of the reef.
   *
   * @return a trigger that is true when the robot is within 1 meter of the reef.
   */
  public boolean isReefInRange() {
    return (getEstimatedPose()
                .getTranslation()
                .getDistance(
                    ReefLocations.getSelectedLocation(getEstimatedPose().getTranslation(), true)
                        .getTranslation())
            <= 1
        || getEstimatedPose()
                .getTranslation()
                .getDistance(
                    ReefLocations.getSelectedLocation(getEstimatedPose().getTranslation(), false)
                        .getTranslation())
            <= 1);
  }

  public boolean isRobotAlignedToLeftReef() {
    return getEstimatedPose()
            .getTranslation()
            .getDistance(
                ReefLocations.getSelectedLocation(getEstimatedPose().getTranslation(), true)
                    .getTranslation())
        <= AutoConstants.AUTO_ALIGN_ACCEPTABLE_ERROR;
  }

  public boolean isRobotAlignedToRightReef() {
    return getEstimatedPose()
            .getTranslation()
            .getDistance(
                ReefLocations.getSelectedLocation(getEstimatedPose().getTranslation(), false)
                    .getTranslation())
        <= AutoConstants.AUTO_ALIGN_ACCEPTABLE_ERROR;
  }

  /**
   * Aligns the robot to the reef.
   *
   * @param left whether to align to the left or right reef.
   */
  public void reefAlign(Supplier<Translation2d> nudgeSupplier) {
    int bestBranchWall = 0;
    boolean bestLeft = true;
    double bestScore = Double.POSITIVE_INFINITY;

    // There are 6 reef walls, and each wall has 2 branches (left/right)
    for (int i = 0; i < 6; i++) {
      for (boolean left : new boolean[] {true, false}) {
        Pose2d branchPose = getBranchPose(i, left);
        Translation2d branchLocation = branchPose.getTranslation();

        // Calculate how far the robot is from the branch
        Translation2d robotToBranchVector =
            branchLocation.minus(poseEstimator.getEstimatedPosition().getTranslation());
        double branchDistanceScore = robotToBranchVector.getNorm();

        // Incorporate any driver input if provided
        Translation2d driverControlVector = nudgeSupplier.get();
        if (AllianceFlipper.isRed()) {
          driverControlVector =
              new Translation2d(-driverControlVector.getX(), -driverControlVector.getY());
        }

        double driverInputScore;
        if (driverControlVector.getNorm() < 0.1) {
          driverInputScore = 0;
        } else {
          double robotToBranchAngle = robotToBranchVector.getAngle().getDegrees();
          double driverControlAngle = driverControlVector.getAngle().getDegrees();
          driverInputScore = driverControlAngle - robotToBranchAngle;
          // Multiply by a factor (here 2) to weight the driver input relative to distance
          driverInputScore = Math.cos(driverInputScore) * 2;
        }

        double branchScore = branchDistanceScore - driverInputScore;

        Logger.recordOutput(
            "Swerve/Reef Align/Wall " + i + " " + (left ? "Left" : "Right") + "/Score",
            branchScore);

        if (branchScore < bestScore) {
          bestScore = branchScore;
          bestBranchWall = i;
          bestLeft = left;
        }
      }
    }
    followRepulsorField(getBranchPose(bestBranchWall, bestLeft), nudgeSupplier);
  }

  /**
   * Aligns the robot to the reef.
   *
   * @param left whether to align to the left or right reef.
   */
  public void reefAlign(boolean left) {
    int bestBranchWall = 0;
    boolean bestLeft = true;
    double bestScore = Double.POSITIVE_INFINITY;

    // There are 6 reef walls, and each wall has 2 branches (left/right)
    for (int i = 0; i < 6; i++) {
      Pose2d branchPose = getBranchPose(i, left);
      Translation2d branchLocation = branchPose.getTranslation();

      // Calculate how far the robot is from the branch
      Translation2d robotToBranchVector =
          branchLocation.minus(poseEstimator.getEstimatedPosition().getTranslation());
      double branchDistanceScore = robotToBranchVector.getNorm();

      double branchScore = branchDistanceScore;

      Logger.recordOutput(
          "Swerve/Reef Align/Wall " + i + " " + (left ? "Left" : "Right") + "/Score", branchScore);

      if (branchScore < bestScore) {
        bestScore = branchScore;
        bestBranchWall = i;
      }
    }
    followRepulsorField(getBranchPose(bestBranchWall, left));
  }

  private Pose2d getBranchPose(int reefWall, boolean left) {
    var branches = AllianceFlipper.isRed() ? ReefLocations.RED_POSES : ReefLocations.BLUE_POSES;
    return branches[reefWall * 2 + (left ? 0 : 1)];
  }
}

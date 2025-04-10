package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.math.mathutils.GeomUtil;
import frc.robot.extras.util.Pose2dMovingAverageFilter;
import frc.robot.extras.util.ThreadManager;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.extras.vision.TigerHelpers;
import frc.robot.extras.vision.TigerHelpers.Botpose;
import frc.robot.extras.vision.TigerHelpers.IMUMode;
import frc.robot.extras.vision.TigerHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import java.util.concurrent.atomic.AtomicReferenceArray;

/**
 * This class is the implementation of the VisionInterface for the physical robot. It uses the
 * ThreadManager to make threads to run the code for processing the vision data from the limelights
 * asynchonously.
 *
 * @author Jack
 * @author Ishan
 */
public class PhysicalVision implements VisionInterface {

  private Pose2d odometryPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;

  private Pose2dMovingAverageFilter pose2dMovingAverageFilter =
      new Pose2dMovingAverageFilter(VisionConstants.POSE_MOVING_AVERAGE_WINDOW_SIZE);

  /** The pose estimates from the limelights in the following order (FRONT_LEFT, FRONT_RIGHT) */
  private final AtomicReferenceArray<MegatagPoseEstimate> limelightEstimates;

  /** The thread manager for the vision threads */
  private final ThreadManager threadManager = new ThreadManager(Limelight.values().length);

  private final boolean[] isMegatag2 = new boolean[Limelight.values().length];

  public PhysicalVision() {
    limelightEstimates = new AtomicReferenceArray<>(Limelight.values().length);
    for (Limelight limelight : Limelight.values()) {
      limelightEstimates.set(limelight.getId(), new MegatagPoseEstimate());
      // Setup port forwarding for each limelight
      setupPortForwarding(limelight);
      // Start a threaded task to check and update the pose for each Limelight
      threadManager.startTask(
          limelight.getName(),
          () -> checkAndUpdatePose(limelight),
          VisionConstants.THREAD_SLEEP_MS);
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // Combine inputs into the main inputs object
    for (Limelight limelight : Limelight.values()) {
      inputs.isLimelightConnected[limelight.getId()] = isLimelightConnected(limelight);

      inputs.limelightSeesAprilTags[limelight.getId()] = canSeeAprilTags(limelight);

      inputs.limelightTargets[limelight.getId()] = getNumberOfAprilTags(limelight);

      inputs.limelightLatencies[limelight.getId()] = getLatencySeconds(limelight);
      inputs.limelightAprilTagDistances[limelight.getId()] =
          getLimelightAprilTagDistance(limelight);
      inputs.limelightTimestamps[limelight.getId()] = getTimestampSeconds(limelight);
      inputs.limelightAmbiguities[limelight.getId()] = getAmbiguity(limelight);

      inputs.limelightCalculatedPoses[limelight.getId()] = getPoseFromAprilTags(limelight);

      inputs.megatag1PoseEstimates[limelight.getId()] = getMegaTag1PoseEstimate(limelight).pose;
      inputs.megatag2PoseEstimates[limelight.getId()] = getMegaTag2PoseEstimate(limelight).pose;

      inputs.isMegaTag2[limelight.getId()] = isMegatag2[limelight.getId()];
    }
  }

  @Override
  public boolean canSeeAprilTags(Limelight limelight) {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // as the limelight can see an april tag but not have it fully in frame, leading
    // to inaccurate pose estimates
    if (isLimelightConnected(limelight)) {
      return Math.abs(TigerHelpers.getTX(limelight.getName())) <= limelight.getAccurateFOV();
    }
    return false;
  }

  @Override
  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).fieldToCamera;
  }

  @Override
  public int getNumberOfAprilTags(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).tagCount;
  }

  @Override
  public double getTimestampSeconds(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).timestampSeconds;
  }

  @Override
  public double getLatencySeconds(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).latency / 1000.0;
  }

  @Override
  public double getLimelightAprilTagDistance(Limelight limelight) {
    if (canSeeAprilTags(limelight)) {
      return limelightEstimates.get(limelight.getId()).avgTagDist;
    }
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  @Override
  public double getAmbiguity(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).ambiguity;
  }

  @Override
  public void setOdometryInfo(
      double headingDegrees, double headingRateDegrees, Pose2d odometryPose) {
    this.headingDegrees = headingDegrees;
    this.headingRateDegreesPerSecond = headingRateDegrees;
    this.odometryPose = odometryPose;
  }

  @Override
  public boolean isValidMeasurement(Limelight limelight) {
    return isValidPoseEstimate(limelight) && isConfident(limelight) && !isTeleporting(limelight);
  }

  /**
   * Gets the pose update of the specified limelight while the robot is enabled.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void enabledPoseUpdate(Limelight limelight) {
    PoseEstimate megatag1Estimate = getMegaTag1PoseEstimate(limelight);
    PoseEstimate megatag2Estimate = getMegaTag2PoseEstimate(limelight);
    if (Math.abs(headingRateDegreesPerSecond) < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE
        && (!isLargeDiscrepancyBetweenTwoPoses(
                limelight,
                VisionConstants.MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD,
                VisionConstants.MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD,
                megatag1Estimate.pose,
                megatag2Estimate.pose)
            || getLimelightAprilTagDistance(limelight)
                > VisionConstants.MEGA_TAG_2_DISTANCE_THRESHOLD)) {
      limelightEstimates.set(
          limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag2Estimate));
      isMegatag2[limelight.getId()] = true;
    } else if (isWithinFieldBounds(megatag1Estimate.pose)) {
      limelightEstimates.set(
          limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag1Estimate));
      isMegatag2[limelight.getId()] = false;
    } else {
      limelightEstimates.set(limelight.getId(), new MegatagPoseEstimate());
      isMegatag2[limelight.getId()] = false;
    }
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void disabledPoseUpdate(Limelight limelight) {
    PoseEstimate megatag1PoseEstimate = getMegaTag1PoseEstimate(limelight);

    limelightEstimates.set(
        limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag1PoseEstimate));
    isMegatag2[limelight.getId()] = false;
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void updatePoseEstimate(Limelight limelight) {
    if (DriverStation.isEnabled()) {
      enabledPoseUpdate(limelight);
    } else {
      disabledPoseUpdate(limelight);
    }
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void checkAndUpdatePose(Limelight limelight) {
    // Megatag 2 uses the gyro orientation to solve for the rotation of the
    // calculated pose. This creates a much more stable and accurate pose when
    // translating, but when rotating but the pose will not be consistent due to
    // latency between receiving and sending measurements.
    TigerHelpers.setRobotOrientation(limelight.getName(), headingDegrees);
    if (isLimelightConnected(limelight) && canSeeAprilTags(limelight)) {
      updateIMUMode(limelight);
      updatePoseEstimate(limelight);
    } else {
      limelightEstimates.set(limelight.getId(), new MegatagPoseEstimate());
    }
  }

  /**
   * Gets the MegaTag1 pose of the robot calculated by specified limelight via any April Tags it
   * sees.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The MegaTag1 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag1PoseEstimate(Limelight limelight) {
    return TigerHelpers.getBotPoseEstimate(limelight.getName(), Botpose.BLUE_MEGATAG1);
  }

  /**
   * Gets the MegaTag2 pose of the robot calculated by specified limelight via any April Tags it
   * sees.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The MegaTag2 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag2PoseEstimate(Limelight limelight) {
    return TigerHelpers.getBotPoseEstimate(limelight.getName(), Botpose.BLUE_MEGATAG2);
  }

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight is connected
   */
  public boolean isLimelightConnected(Limelight limelight) {
    return TigerHelpers.getLimelightNetworkTable(limelight.getName()).containsKey("tv");
  }

  /**
   * Stops the thread for the specified limelight.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void stopLimelightThread(Limelight limelight) {
    threadManager.stopThread(limelight.getName());
  }

  /** Shuts down all the threads. */
  public void endAllThreads() {
    threadManager.shutdownAllThreads();
  }

  private int iteration = 0;

  private void updateIMUMode(Limelight limelight) {
    iteration++;
    if (limelight.hasInternalIMU()) {
      if (DriverStation.isEnabled()) {
        // Enable internal IMU for better pose accuracy when enabled
        if (iteration % 20 == 0) {
          TigerHelpers.setIMUMode(limelight.getName(), IMUMode.EXTERNAL_IMU_SEED_INTERNAL);
        } else {
          TigerHelpers.setIMUMode(limelight.getName(), IMUMode.INTERNAL_EXTERNAL_ASSISTED);
        }
        TigerHelpers.setLimelightThrottle(limelight.getName(), VisionConstants.ENABLED_THROTTLE);
      } else {
        // Disable internal IMU when robot is disabled
        TigerHelpers.setIMUMode(limelight.getName(), IMUMode.EXTERNAL_IMU_SEED_INTERNAL);
        TigerHelpers.setLimelightThrottle(limelight.getName(), VisionConstants.DISABLED_THROTTLE);
      }
    }
  }

  /**
   * Checks if the robot is teleporting based on the pose estimate from the limelight. This method
   * needs to be synchronized to prevent race conditions when accessing the
   * pose2dMovingAverageFilter.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the robot is teleporting, false otherwise
   */
  private synchronized boolean isTeleporting(Limelight limelight) {
    return !GeomUtil.arePosesWithinThreshold(
        VisionConstants.MAX_TRANSLATION_DELTA_METERS,
        VisionConstants.MAX_ROTATION_DELTA_DEGREES,
        getPoseFromAprilTags(limelight),
        pose2dMovingAverageFilter.calculate(getPoseFromAprilTags(limelight)));
  }

  /**
   * Checks if the pose estimate exists and whether it is within the field.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the pose estimate exists within the field and the pose estimate is not null
   */
  private boolean isValidPoseEstimate(Limelight limelight) {
    return getPoseFromAprilTags(limelight) != new Pose2d()
        && isWithinFieldBounds(getPoseFromAprilTags(limelight));
  }

  /**
   * Checks whether the pose estimate is within the field
   *
   * @param poseEstimate The pose estimate to check
   */
  private boolean isWithinFieldBounds(Pose2d poseEstimate) {
    double minX = DriveConstants.TRACK_WIDTH / 2.0;
    double maxX = FieldConstants.FIELD_LENGTH_METERS - DriveConstants.TRACK_WIDTH / 2.0;
    double minY = DriveConstants.WHEEL_BASE / 2.0;
    double maxY = FieldConstants.FIELD_WIDTH_METERS - DriveConstants.WHEEL_BASE / 2.0;
    return (poseEstimate.getX() > minX && poseEstimate.getX() < maxX)
        && (poseEstimate.getY() > minY && poseEstimate.getY() < maxY);
  }

  /**
   * Checks if there is a large discrepancy between two poses. This is used to determine if the
   * estimated megatag 1 and 2 pose are within a certain threshold, whether or not they are within
   * this threshold helps determine which pose to use.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @param translationThresholdMeters The translation threshold in meters.
   * @param rotationThresholdDegrees The rotation threshold in degrees.
   * @param pose1 The first pose to compare.
   * @param pose2 The second pose to compare.
   * @return True if the discrepancy between the two poses is greater than the threshold, false.
   */
  private boolean isLargeDiscrepancyBetweenTwoPoses(
      Limelight limelight,
      double translationThresholdMeters,
      double rotationThresholdDegrees,
      Pose2d pose1,
      Pose2d pose2) {
    return !GeomUtil.arePosesWithinThreshold(
        translationThresholdMeters, rotationThresholdDegrees, pose1, pose2);
  }

  /**
   * Checks if the limelight is confident in its pose estimate
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight is confident in its pose estimate, false otherwise
   */
  private boolean isConfident(Limelight limelight) {
    return getAmbiguity(limelight) <= VisionConstants.MAX_AMBIGUITY_THRESHOLD;
  }

  /**
   * Sets up port forwarding for the specified Limelight. This method forwards a range of ports from
   * the robot to the Limelight, allowing network communication between the robot and the Limelight.
   *
   * <p>Each Limelight is assigned a unique port offset based on its ID. The method forwards ports
   * 5800 to 5809 for each Limelight, with the port offset applied to each port number. For example,
   * if the Limelight ID is 1, the ports 5810 to 5819 will be forwarded.
   *
   * @param limelight The Limelight for which to set up port forwarding.
   */
  private void setupPortForwarding(Limelight limelight) {
    int portOffset = VisionConstants.PORT_OFFSET * limelight.getId();
    for (int port = VisionConstants.BASE_PORT;
        port < VisionConstants.BASE_PORT + VisionConstants.PORT_RANGE;
        port++) {
      PortForwarder.add(
          port + portOffset, limelight.getName() + VisionConstants.LIMELIGHT_DOMAIN, port);
    }
  }
}

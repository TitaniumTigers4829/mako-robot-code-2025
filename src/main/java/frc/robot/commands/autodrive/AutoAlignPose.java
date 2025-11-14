// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** This command is used to align the robot to a specific pose using the vision subsystem. */
public class AutoAlignPose extends DriveCommandBase {
  private final SwerveDrive swerveDrive;
  private final Pose2d pose;

  private Pose2d currentPose;
  private Pose2d desiredPose;

  private final Consumer<Boolean> isAligned;

  private ProfiledPIDController xTranslationController =
      new ProfiledPIDController(4.8, 0, 0.29, new Constraints(3.0, 2)); // 2
  private ProfiledPIDController yTranslationController =
      new ProfiledPIDController(4.8, 0, 0.29, new Constraints(3.0, 2));
  private ProfiledPIDController rotationController =
      new ProfiledPIDController(4.8, 0, 0.29, new Constraints(2, 4));

  /** Creates a new AutoAlignPose. */
  public AutoAlignPose(
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      Pose2d pose,
      Consumer<Boolean> isAligned) {
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    this.pose = pose;
    this.isAligned = isAligned;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = swerveDrive.getEstimatedPose();

    desiredPose = pose;

    // TODO: reset vels here
    xTranslationController.reset(currentPose.getX());
    yTranslationController.reset(currentPose.getY());
    rotationController.reset(currentPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    currentPose = swerveDrive.getEstimatedPose();

    double xOutput = xTranslationController.calculate(currentPose.getX(), desiredPose.getX());
    double yOutput = yTranslationController.calculate(currentPose.getY(), desiredPose.getY());
    double rotationOutput =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            MathUtil.clamp(xOutput, -.75, .75),
            MathUtil.clamp(yOutput, -.75, .75),
            rotationOutput,
            swerveDrive.getOdometryRotation2d());

    swerveDrive.drive(outputSpeeds.unaryMinus(), false);

    Logger.recordOutput("AutoAlign/Setpoint", desiredPose);
    Logger.recordOutput("AutoAlign/Error", desiredPose.minus(currentPose));
    Logger.recordOutput("AutoAlign/OutputSpeeds", outputSpeeds.unaryMinus());
    Logger.recordOutput(
        "AutoAlign/ProfileVelocityX", -xTranslationController.getSetpoint().velocity);
    Logger.recordOutput(
        "AutoAlign/ProfileVelocityY", -yTranslationController.getSetpoint().velocity);
    Logger.recordOutput(
        "AutoAlign/ProfileVelocityHeading", rotationController.getSetpoint().velocity);
    isAligned.accept(swerveDrive.isReefInRange());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return currentPose.getTranslation().getDistance(pose.getTranslation()) <= 0.02
        && Math.abs(swerveDrive.getChassisSpeeds().vxMetersPerSecond) < 0.075
        && Math.abs(swerveDrive.getChassisSpeeds().vyMetersPerSecond) < 0.075;
  }
}

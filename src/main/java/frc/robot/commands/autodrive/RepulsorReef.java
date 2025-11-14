// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autodrive;

import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RepulsorReef extends DriveCommandBase {
  SwerveDrive swerveDrive;
  boolean left;

  /** Creates a new RepulsorReef. */
  public RepulsorReef(SwerveDrive swerveDrive, VisionSubsystem visionSubsystem, boolean left) {
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    this.left = left;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    swerveDrive.reefAlign(left);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return swerveDrive.isReefInRange()
        && Math.abs(swerveDrive.getChassisSpeeds().vxMetersPerSecond) < 0.075
        && Math.abs(swerveDrive.getChassisSpeeds().vyMetersPerSecond) < 0.075;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;

public class SetFunnelAngle extends Command {
  private final FunnelSubsystem funnelSubsystem;
  private double angle;

  public SetFunnelAngle(FunnelSubsystem funnelSubsystem, double angle) {
    this.funnelSubsystem = funnelSubsystem;
    this.angle = angle;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSubsystem.setFunnelAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

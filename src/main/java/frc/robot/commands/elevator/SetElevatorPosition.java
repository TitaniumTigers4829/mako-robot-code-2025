package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private double position;

  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(ElevatorSubsystem elevatorSubsystem, double position) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtSetpoint(position);
  }
}

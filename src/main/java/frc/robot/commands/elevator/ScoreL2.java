package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreL2 extends SequentialCommandGroup {
  /** Creates a new ScoreL2. */
  public ScoreL2(ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
    addCommands(
        elevatorSubsystem.setElevationPosition(ElevatorSetpoints.L2.getPosition()),
        coralIntakeSubsystem.ejectCoral());
  }
}

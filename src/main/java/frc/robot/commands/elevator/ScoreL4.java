package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreL4 extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4(ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
    addCommands(
        elevatorSubsystem.setElevationPosition(ElevatorSetpoints.L4.getPosition()).withTimeout(6.0),
        Commands.run(
                () -> coralIntakeSubsystem.setIntakeVelocity(CoralIntakeConstants.EJECT_SPEED),
                coralIntakeSubsystem)
            .until(() -> !coralIntakeSubsystem.hasCoral()));
  }
}

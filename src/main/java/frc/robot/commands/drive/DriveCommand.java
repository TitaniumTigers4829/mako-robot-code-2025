package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommand extends DriveCommandBase {

  private final SwerveDrive driveSubsystem;

  private final DoubleSupplier leftJoystickX, leftJoystickY, rightJoystickX;
  private final BooleanSupplier isFieldRelative, isHighRotation;
  private double angularSpeed;
  private final Consumer<Boolean> isAligned;

  /**
   * The command for driving the robot using joystick inputs.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftJoystickY The joystick input for driving forward and backwards
   * @param leftJoystickX The joystick input for driving left and right
   * @param rightJoystickX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive field relative
   * @param isHighRotation The boolean supplier for if the robot should drive with a higher rotation
   */
  public DriveCommand(
      SwerveDrive driveSubsystem,
      VisionSubsystem visionSubsystem,
      DoubleSupplier leftJoystickX,
      DoubleSupplier leftJoystickY,
      DoubleSupplier rightJoystickX,
      BooleanSupplier isFieldRelative,
      BooleanSupplier isHighRotation,
      Consumer<Boolean> isAligned) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.isAligned = isAligned;
    addRequirements(driveSubsystem, visionSubsystem);

    this.leftJoystickY = leftJoystickY;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
    this.isHighRotation = isHighRotation;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    Supplier<Translation2d> driveTranslationalControlSupplier =
        () -> {
          return new Translation2d(leftJoystickX.getAsDouble(), leftJoystickY.getAsDouble());
        };
    // Most of the time the driver prefers that the robot rotates slowly, as it gives them more
    // control
    // but sometimes (e.g. when fighting defense bots) being able to rotate quickly is necessary
    if (isHighRotation.getAsBoolean()) {
      angularSpeed = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    } else {
      angularSpeed = DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND;
    }

    // Drives the robot by scaling the joystick inputs
    driveSubsystem.drive(
        leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        rightJoystickX.getAsDouble() * angularSpeed,
        isFieldRelative.getAsBoolean());
    // Runs all the code from DriveCommand that estimates pose
    super.execute();
    isAligned.accept(driveSubsystem.isReefInRange());
  }

  @Override
  public void end(boolean interrupted) {
    angularSpeed = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

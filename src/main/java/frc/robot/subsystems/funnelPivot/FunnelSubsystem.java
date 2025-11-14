package frc.robot.subsystems.funnelPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.logging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends SubsystemBase {
  private final FunnelPivotInterface funnelPivotInterface;
  private final FunnelPivotInputsAutoLogged inputs = new FunnelPivotInputsAutoLogged();

  private static final LoggedTunableNumber funnelP = new LoggedTunableNumber("Funnel/FunnelP");

  static {
    funnelP.initDefault(FunnelConstants.PIVOT_P);
  }

  public FunnelSubsystem(FunnelPivotInterface funnelPivotInterface) {
    this.funnelPivotInterface = funnelPivotInterface;
  }

  public void setFunnelSpeed(double speed) {
    funnelPivotInterface.setFunnelSpeed(speed);
  }

  public void setFunnelAngle(double angle) {
    funnelPivotInterface.setFunnelAngle(angle);
  }

  public void setFunnelVoltage(double voltage) {
    funnelPivotInterface.setFunnelVoltage(voltage);
  }

  public void periodic() {
    funnelPivotInterface.updateInputs(inputs);
    Logger.processInputs("FunnelPivotSubsystem/", inputs);
    // Check for alerts
    funnelPivotInterface.checkAlerts();
    // Update tunable numbers
    if (funnelP.hasChanged(hashCode())) {
      funnelPivotInterface.setPID(funnelP.get(), 0, 0);
    }
  }

  public Command manualFunnel(DoubleSupplier position) {
    return new RunCommand(() -> funnelPivotInterface.setFunnelSpeed(position.getAsDouble()), this);
  }

  public Command dropFunnel() {
    return new RunCommand(() -> funnelPivotInterface.setFunnelSpeed(0.3), this)
        .withTimeout(0.5)
        .andThen(() -> funnelPivotInterface.setFunnelSpeed(0));
  }
}

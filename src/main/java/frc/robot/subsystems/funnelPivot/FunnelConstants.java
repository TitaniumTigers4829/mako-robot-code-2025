package frc.robot.subsystems.funnelPivot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class FunnelConstants {
  public static final int FUNNEL_PIVOT_MOTOR_ID = 34;
  public static final int FUNNEL_ENCODER_ID = 33; // Choose an appropriate CAN ID

  public static final SensorDirectionValue FUNNEL_ENCODER_REVERSED =
      SensorDirectionValue.Clockwise_Positive;
  public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 100.0;
  public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND = 100.0;

  public static final double MAX_ANGLE = FunnelConstants.ANGLE_ZERO;
  public static final double MIN_ANGLE = FunnelConstants.ANGLE_INTAKE;

  public static final double ANGLE_ZERO = 0.0;
  public static final double ANGLE_INTAKE = -50.083;
  public static final double PIVOT_P = 5.0;
  public static final double PIVOT_I = 0.0;
  public static final double PIVOT_D = 0.0;
  public static final double PIVOT_G = 0.0;

  public static final double FUNNEL_GEAR_RATIO = 0.375; // 24 to 64 flame unc zimo if wrong
  public static final double FUNNEL_PIVOT_MASS = 0.0;
  public static final double FUNNEL_PIVOT_LENGTH = 0.0;

  public static final double FUNNEL_PIVOT_ANGLE = 0.0;
  public static final double FUNNEL_PIVOT_SPEED = 0.0;
  public static final double FUNNEL_VOLTAGE = 0.0;
  public static final double FUNNEL_NEUTRAL_SPEED = 0.0;

  public static final double FUNNEL_ZERO_ANGLE = 0.268;
  public static final SensorDirectionValue FUNNEL_ENCODER_DIRECTION =
      SensorDirectionValue.Clockwise_Positive;
}

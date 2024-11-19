package frc.robot.subsystems.maxPosition1;

import org.littletonrobotics.junction.AutoLog;

public interface MaxPosition1IO {
  @AutoLog
  public static class MaxPosition1IOInputs {
    public double internalPositionRad = 0.0;
    public double internalVelocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MaxPosition1IOInputs inputs) {}

  /** Run the arm1 motor(s) at the specified voltages. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean maxPosition1Brake) {}
}

package frc.robot.subsystems.maxVelocity2;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class MaxVelocity2 extends SubsystemBase {
  private final MaxVelocity2IO io;
  private final MaxVelocity2IOInputsAutoLogged inputs = new MaxVelocity2IOInputsAutoLogged();
  private SimpleMotorFeedforward ffModel;

  public LoggedDashboardNumber maxVelocity2VelocityInput =
      new LoggedDashboardNumber("MaxVelocity2 RPM", 0);

  public LoggedDashboardNumber elasticVolts2 =
      new LoggedDashboardNumber("MaxVelocity2ElasticVolts", 0);

  public LoggedDashboardBoolean isvolts2 =
      new LoggedDashboardBoolean("Run RPM             Run Volts", true);

  public LoggedDashboardBoolean isBrake2 = new LoggedDashboardBoolean("Brake Mode", false);
  boolean brakeNeedsUpdate = false;

  public LoggedDashboardNumber appliedVoltage2 = new LoggedDashboardNumber("Applied Volts", 0);

  public LoggedDashboardNumber velocityRPM2 =
      new LoggedDashboardNumber("Current MaxVelocity2 RPM", 0);

  public LoggedDashboardNumber setKP2 = new LoggedDashboardNumber("Set kP", 0.0001);

  public LoggedDashboardNumber setKI2 = new LoggedDashboardNumber("Set kI", 0.0);

  public LoggedDashboardNumber setKD2 = new LoggedDashboardNumber("Set kD", 0.0);

  public LoggedDashboardNumber setkS2 = new LoggedDashboardNumber("Set kS", 0.0);
  public LoggedDashboardNumber setkV2 = new LoggedDashboardNumber("Set kV", 0.0043);

  /** Creates a new MaxVelocity2. */
  public MaxVelocity2(MaxVelocity2IO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0043);
        io.configurePID(0.0001, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0043);
        io.configurePID(0.0001, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0043);
        io.configurePID(0.0001, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("MaxVelocity2", inputs);

    if (isvolts2.get()) {
      double motorVoltage = elasticVolts2.get();
      runVolts(motorVoltage);
    } else {
      double motorRPM = maxVelocity2VelocityInput.get();
      runVelocity(motorRPM);
    }

    if (!(brakeNeedsUpdate == isBrake2.get())) {
      brakeMode(isBrake2.get() ? IdleMode.kBrake : IdleMode.kCoast);
      brakeNeedsUpdate = !brakeNeedsUpdate;
    }

    appliedVoltage2.set(inputs.appliedVolts);

    velocityRPM2.set(Math.round(getVelocityRPM()));

    io.configurePID(setKP2.get(), setKI2.get(), setKD2.get());

    ffModel = new SimpleMotorFeedforward(setkS2.get(), setkV2.get());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM2) {
    io.setVelocity(velocityRPM2, ffModel.calculate(velocityRPM2));

    Logger.recordOutput("MaxVelocity2/SetpointRPM", velocityRPM2);
    Logger.recordOutput("MaxVelocity2/ffVolts", ffModel.calculate(velocityRPM2));
  }

  /** Stops the convey. */
  public void stop() {
    io.stop();
  }

  public void brakeMode(IdleMode isBrake2) {
    io.brakeMode(isBrake2);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return inputs.velocityRPM2;
  }
}

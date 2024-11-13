package frc.robot.subsystems.maxVelocity1;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class MaxVelocity1 extends SubsystemBase {
  private final MaxVelocity1IO io;
  private final MaxVelocity1IOInputsAutoLogged inputs = new MaxVelocity1IOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final DigitalInput maxVelocity1NoteSensor = new DigitalInput(9);

  public LoggedDashboardNumber maxVelocity1VelocityInput =
      new LoggedDashboardNumber("MaxVelocity1 RPM", 0);

  public LoggedDashboardNumber elasticVolts =
      new LoggedDashboardNumber("MaxVelocity1ElasticVolts", 0);

  public LoggedDashboardBoolean isvolts =
      new LoggedDashboardBoolean("Run RPM             Run Volts", true);

  public LoggedDashboardBoolean isBrake = new LoggedDashboardBoolean("Brake Mode", false);
  boolean brakeNeedsUpdate = false;

  public LoggedDashboardNumber appliedVoltage = new LoggedDashboardNumber("Applied Volts", 0);

  public LoggedDashboardNumber velocityRPM =
      new LoggedDashboardNumber("Current MaxVelocity1 RPM", 0);

  public LoggedDashboardNumber setKP = new LoggedDashboardNumber("Set kP", 0);
  boolean kPNeedsUpdate = false;

  public LoggedDashboardNumber setKI = new LoggedDashboardNumber("Set kI", 0);
  boolean kINeedsUpdate = false;

  public LoggedDashboardNumber setKD = new LoggedDashboardNumber("Set kD", 0);
  boolean kDNeedsUpdate = false;

  /** Creates a new MaxVelocity1. */
  public MaxVelocity1(MaxVelocity1IO io) {
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
    Logger.processInputs("MaxVelocity1", inputs);

    if (isvolts.get()) {
      double motorVoltage = elasticVolts.get();
      runVolts(motorVoltage);
    } else {
      double motorRPM = maxVelocity1VelocityInput.get();
      runVelocity(motorRPM);
    }

    if (!(brakeNeedsUpdate == isBrake.get())) {
      brakeMode(isBrake.get() ? IdleMode.kBrake : IdleMode.kCoast);
      brakeNeedsUpdate = !brakeNeedsUpdate;
    }

    appliedVoltage.set(inputs.appliedVolts);

    velocityRPM.set(Math.round(getVelocityRPM()));

    io.configurePID(setKP.get(), setKI.get(), setKD.get());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    io.setVelocity(velocityRPM, ffModel.calculate(velocityRPM));

    Logger.recordOutput("MaxVelocity1/SetpointRPM", velocityRPM);
    Logger.recordOutput("MaxVelocity1/ffVolts", ffModel.calculate(velocityRPM));
  }

  /** Stops the convey. */
  public void stop() {
    io.stop();
  }

  public void brakeMode(IdleMode isBrake) {
    io.brakeMode(isBrake);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  // The note sensor reads True without a note present and False when it sees a note
  @AutoLogOutput
  public boolean noteIsLoaded() {
    return !maxVelocity1NoteSensor.get();
  }

  // Run conveyor until the Note sensor detects the Note
  // There is no simulation of the Note sensor, so just stop the simulated maxVelocity1or
  // immediately
  public Command loadCommand() {
    return new StartEndCommand(
            () -> runVelocity(maxVelocity1VelocityInput.get()), () -> stop(), this)
        .until(() -> (noteIsLoaded() || RobotBase.isSimulation()))
        .withName("Load");
  }
}

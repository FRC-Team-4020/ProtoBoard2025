package frc.robot.subsystems.MaxPosition1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedMaxPosition1Sim;
import frc.robot.Constants.MaxPosition1Constants;

public class MaxPosition1IOSim implements MaxPosition1IO {

  private double LOOP_PERIOD_SEC = 0.02;

  private SingleJointedMaxPosition1Sim MaxPosition1Sim;
  private boolean lastEnabled = false;
  private double appliedVolts = 0.0;

  public MaxPosition1IOSim() {

    MaxPosition1Sim =
        new SingleJointedMaxPositionSim(
            DCMotor.getNeoVortex(1),
            MaxPosition1Constants.ARM_GEAR_REDUCTION,
            SingleJointedMaxPosition1Sim.estimateMOI(
                Units.inchesToMeters(MaxPosition1Constants.ARM_LENGTH_IN),
                Units.lbsToKilograms(MaxPosition1Constants.ARM_MASS_LBF)),
            Units.inchesToMeters(MaxPosition1Constants.ARM_LENGTH_IN),
            Units.degreesToRadians(MaxPosition1Constants.ARM_MIN_ANGLE_DEG),
            Units.degreesToRadians(MaxPosition1Constants.ARM_MAX_ANGLE_DEG),
            true,
            Units.degreesToRadians(MaxPosition1Constants.ARM_MIN_ANGLE_DEG));

    MaxPosition1Sim.setState(VecBuilder.fill(Units.degreesToRadians(MaxPosition1Constants.ARM_MIN_ANGLE_DEG), 0.0));
  }

  @Override
  public void updateInputs(MaxPosition1IOInputs inputs) {
    // Reset voltage when disabled
    if (DriverStation.isDisabled()) {
      setVoltage(0.0);
    }

    // Reset position on enable
    if (DriverStation.isEnabled() && !lastEnabled) {
      maxPosition1Sim.setState(
          VecBuilder.fill(Units.degreesToRadians(MaxPosition1Constants.ARM_MIN_ANGLE_DEG), 0.0));
    }
    lastEnabled = DriverStation.isEnabled();

    // Update sim state
    maxPosition1Sim.update(LOOP_PERIOD_SEC);

    // Log sim data
    inputs.internalPositionRad = maxPosition1Sim.getAngleRads();
    inputs.internalVelocityRadPerSec = maxPosition1Sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = maxPosition1Sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    maxPosition1Sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}

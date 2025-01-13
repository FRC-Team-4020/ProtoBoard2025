package frc.robot.subsystems.arm1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Arm1Constants;

public class Arm1IOSim implements Arm1IO {

  private double LOOP_PERIOD_SEC = 0.02;

  private SingleJointedArmSim arm1Sim;
  private boolean lastEnabled = false;
  private double appliedVolts = 0.0;

  public Arm1IOSim() {

    arm1Sim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            Arm1Constants.ARM_GEAR_REDUCTION,
            SingleJointedArmSim.estimateMOI(
                Units.inchesToMeters(Arm1Constants.ARM_LENGTH_IN),
                Units.lbsToKilograms(Arm1Constants.ARM_MASS_LBF)),
            Units.inchesToMeters(Arm1Constants.ARM_LENGTH_IN),
            Units.degreesToRadians(Arm1Constants.ARM_POS_0),
            Units.degreesToRadians(Arm1Constants.ARM_POS_1),
            true,
            Units.degreesToRadians(Arm1Constants.ARM_POS_0));

    arm1Sim.setState(VecBuilder.fill(Units.degreesToRadians(Arm1Constants.ARM_POS_0), 0.0));
  }

  @Override
  public void updateInputs(Arm1IOInputs inputs) {
    // Reset voltage when disabled
    if (DriverStation.isDisabled()) {
      setVoltage(0.0);
    }

    // Reset position on enable
    if (DriverStation.isEnabled() && !lastEnabled) {
      arm1Sim.setState(VecBuilder.fill(Units.degreesToRadians(Arm1Constants.ARM_POS_0), 0.0));
    }
    lastEnabled = DriverStation.isEnabled();

    // Update sim state
    arm1Sim.update(LOOP_PERIOD_SEC);

    // Log sim data
    inputs.internalPositionRad = arm1Sim.getAngleRads();
    inputs.internalVelocityRadPerSec = arm1Sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = arm1Sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    arm1Sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}

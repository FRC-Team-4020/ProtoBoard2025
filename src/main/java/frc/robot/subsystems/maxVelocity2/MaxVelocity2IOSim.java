package frc.robot.subsystems.maxVelocity2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MaxVelocity2IOSim implements MaxVelocity2IO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNeo550(1), 4.0, 0.0001);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(MaxVelocity2IOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM()) + ffVolts, -12.0, 12.0);
    }

    sim.setInputVoltage(appliedVolts);

    sim.update(0.02);

    inputs.positionRot = 0.0;
    inputs.velocityRPM2 = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRPM2, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRPM2);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public void brakeMode(boolean isBrake2) {
    this.brakeMode(isBrake2);
  }
}

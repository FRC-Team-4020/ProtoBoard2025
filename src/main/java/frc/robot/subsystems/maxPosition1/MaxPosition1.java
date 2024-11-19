package frc.robot.subsystems.maxPosition11;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.MaxPosition1Feedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MaxPosition1Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class MaxPosition1 extends SubsystemBase {
  public LoggedDashboardNumber elasticRotations =
      new LoggedDashboardNumber("MaxPosition1ElasticRotations", 300);
  private final MaxPosition1IO io;
  private final Maxposition1IOInputsAutoLogged inputs = new MaxPosition1IOInputsAutoLogged();
  private final MaxPosition1Feedforward ffModel;
  private final ProfiledPIDController pid;
  private Double pidOutput;
  private Double feedforwardOutput;
  private Double angleGoalRad = Units.degreesToRadians(MaxPosition1Constants.MaxPosition1_TARGET_DEG);
  private String MaxPosition1Status = "#000000";
  public Boolean MaxPosition1ClosedLoop = false;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d MaxPosition1Pivot = mech2d.getRoot("MaxPosition1Pivot", 30, 30);
  private final MechanismLigament2d MaxPosition1Tower =
      MaxPosition1Pivot.append(new MechanismLigament2d("MaxPosition1Tower", 30, -90));
  private final MechanismLigament2d MaxPosition1 =
      MaxPosition1Pivot.append(
          new MechanismLigament2d(
              "MaxPosition1",
              30,
              Units.radiansToDegrees(inputs.internalPositionRad),
              6,
              new Color8Bit(Color.kYellow)));

  /** Creates a new Arm. */
  public MaxPosition11(MaxPosition11IO io) {
    this.io = io;

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("MaxPosition1 Sim", mech2d);
    MaxPosition1Tower.setColor(new Color8Bit(Color.kBlue));

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new MaxPosition1Feedforward(0.0, 0.16, 6.6, 0.01);
        pid = new ProfiledPIDController(60.0, 0.0, 1.0, new TrapezoidProfile.Constraints(1.8, 4.0));
        break;
      case REPLAY:
        ffModel = new MaxPosition1Feedforward(0.0, 0.21, 6.93, 0.01);
        pid = new ProfiledPIDController(80.0, 0.0, 3.0, new TrapezoidProfile.Constraints(1.7, 8.0));
        break;
      case SIM:
        ffModel = new MaxPosition1Feedforward(0.0, 0.21, 6.93, 0.01);
        pid = new ProfiledPIDController(80.0, 0.0, 3.0, new TrapezoidProfile.Constraints(1.7, 8.0));
        break;
      default:
        ffModel = new MaxPosition1Feedforward(0.0, 0.0, 0.0, 0.0);
        pid = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        break;
    }
    pid.setTolerance(Units.degreesToRadians(MaxPosition1Constants.MaxPosition1_ANGLE_TOLERANCE_DEG));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("MaxPosition1", inputs);

    double rotations = elasticRotations.get();
    setGoalDeg(rotations);

    if (MaxPosition1ClosedLoop) {
      pidOutput = pid.calculate(inputs.internalPositionRad, angleGoalRad);
      feedforwardOutput = ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);

      Logger.recordOutput("MaxPosition1/atGoal", pid.atGoal());
      Logger.recordOutput("MaxPosition1/angleGoalRad", angleGoalRad);
      Logger.recordOutput("MaxPosition1/angleSPRad", pid.getSetpoint().position);
      Logger.recordOutput("MaxPosition1/angleSPRadPerSec", pid.getSetpoint().velocity);
      Logger.recordOutput("MaxPosition1/feedbackOP", pidOutput);
      Logger.recordOutput("MaxPosition1/feedforwardOP", feedforwardOutput);

    } else {
      pid.reset(inputs.internalPositionRad);
    }

    if (inputs.internalPositionRad < -0.523) {
      MaxPosition1Status = "#4CAF50"; // green
    } else if (inputs.internalPositionRad < 0.872) {
      MaxPosition1Status = "#F44336"; // red
    } else {
      MaxPosition1Status = "#9528CC"; // purple
    }

    Logger.recordOutput("MaxPosition1/angleInternalRad", inputs.internalPositionRad);
    Logger.recordOutput("MaxPosition1/angleInternalDeg", Units.radiansToDegrees(inputs.internalPositionRad));
    Logger.recordOutput("MaxPosition1/motorVolts", inputs.appliedVolts);
    Logger.recordOutput("MaxPosition1/MaxPosition1IsUp", MaxPosition1IsUp());
    Logger.recordOutput("MaxPosition1/status", MaxPosition1Status);

    // Update the Mechanism Arm angle based on the simulated arm angle
    MaxPosition1.setAngle(Units.radiansToDegrees(inputs.internalPositionRad));
  }

  /** Set the goal for the arm angle. Put controller in auto if not already. */
  public void setGoalDeg(double setpointDeg) {
    MaxPosition1ClosedLoop = true;
    angleGoalRad =
        Units.degreesToRadians(
            MathUtil.clamp(
                setpointDeg, MaxPosition1Constants.MaxPosition1_MIN_ANGLE_DEG, MaxPosition1Constants.MaxPosition1_MAX_ANGLE_DEG));
  }

  /** Stops the Arm. */
  public void stop() {
    io.stop();
    MaxPosition1ClosedLoop = false;
  }

  /** Returns the current arm angle in degrees. */
  @AutoLogOutput(key = "MaxPosition1/angleInternalDeg")
  public double getPositionDeg() {
    return Units.radiansToDegrees(inputs.internalPositionRad);
  }

  // Check if arm is within a tolerance of the down/load position.
  // Typically used as a permissive in commands.
  public boolean MaxPosition1AtTarget() {
    return Units.radiansToDegrees(inputs.internalPositionRad)
        <= MaxPosition1Constants.MaxPosition1_TARGET_DEG + MaxPosition1Constants.MaxPosition1_IS_DOWN_TOLERANCE_DEG;
  }

  public boolean MaxPosition1AtZero() {
    return Units.radiansToDegrees(inputs.internalPositionRad)
        <= MaxPosition1Constants.MaxPosition1_IS_DOWN_TOLERANCE_DEG;
  }

  // Define "arm up" as an angle greater than 0 radians - currently no
  // shooting position would be lower than that
  public boolean MaxPosition1IsUp() {
    return inputs.internalPositionRad >= 0.0;
  }

  // If the arm is less than 75 degrees it is not in an amp position
  public boolean MaxPosition1IsNotAmped() {
    return inputs.internalPositionRad < Units.degreesToRadians(75.0);
  }

  // Has the arm reached the closed-loop goal?
  // Tolerance is set separately with ARM_ANGLE_TOLERANCE_DEG.
  public boolean atGoal() {
    return pid.atGoal();
  }

  // Move the arm to the stow/loading position.
  public Command MaxPosition1ToTargetCommand() {
    return new RunCommand(() -> setGoalDeg(MaxPosition1Constants.MaxPosition1_TARGET_DEG)).until(() -> MaxPosition1AtTarget());
  }

  public Command MaxPosition1ToZeroCommand() {
    return new RunCommand(() -> setGoalDeg(0.0)).until(() -> MaxPosition1AtZero());
  }
}

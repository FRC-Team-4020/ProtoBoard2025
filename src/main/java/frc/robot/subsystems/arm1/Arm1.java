package frc.robot.subsystems.arm1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm1Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Arm1 extends SubsystemBase {
  public LoggedDashboardNumber elasticRotations =
      new LoggedDashboardNumber("Arm1ElasticRotations", 300);
  private final Arm1IO io;
  private final Arm1IOInputsAutoLogged inputs = new Arm1IOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final ProfiledPIDController pid;
  private Double pidOutput;
  private Double feedforwardOutput;
  private Double angleGoalRad = Units.degreesToRadians(Arm1Constants.ARM_TARGET_DEG);
  private String Arm1Status = "#000000";
  public Boolean Arm1ClosedLoop = false;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d Arm1Pivot = mech2d.getRoot("Arm1Pivot", 30, 30);
  private final MechanismLigament2d Arm1Tower =
      Arm1Pivot.append(new MechanismLigament2d("Arm1Tower", 30, -90));
  private final MechanismLigament2d Arm1 =
      Arm1Pivot.append(
          new MechanismLigament2d(
              "Arm1",
              30,
              Units.radiansToDegrees(inputs.internalPositionRad),
              6,
              new Color8Bit(Color.kYellow)));

  /** Creates a new Arm. */
  public Arm1(Arm1IO io) {
    this.io = io;

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm1 Sim", mech2d);
    Arm1Tower.setColor(new Color8Bit(Color.kBlue));

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0.0, 6.6, 0.01);
        pid = new ProfiledPIDController(60.0, 0.0, 1.0, new TrapezoidProfile.Constraints(1.8, 4.0));
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 6.93, 0.01);
        pid = new ProfiledPIDController(80.0, 0.0, 3.0, new TrapezoidProfile.Constraints(1.7, 8.0));
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 6.93, 0.01);
        pid = new ProfiledPIDController(80.0, 0.0, 3.0, new TrapezoidProfile.Constraints(1.7, 8.0));
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        pid = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        break;
    }
    pid.setTolerance(Units.degreesToRadians(Arm1Constants.ARM_ANGLE_TOLERANCE_DEG));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm1", inputs);

    double rotations = elasticRotations.get();
    setGoalDeg(rotations);

    if (Arm1ClosedLoop) {
      pidOutput = pid.calculate(inputs.internalPositionRad, angleGoalRad);
      feedforwardOutput = ffModel.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);

      Logger.recordOutput("Arm1/atGoal", pid.atGoal());
      Logger.recordOutput("Arm1/angleGoalRad", angleGoalRad);
      Logger.recordOutput("Arm1/angleSPRad", pid.getSetpoint().position);
      Logger.recordOutput("Arm1/angleSPRadPerSec", pid.getSetpoint().velocity);
      Logger.recordOutput("Arm1/feedbackOP", pidOutput);
      Logger.recordOutput("Arm1/feedforwardOP", feedforwardOutput);

    } else {
      pid.reset(inputs.internalPositionRad);
    }

    if (inputs.internalPositionRad < -0.523) {
      Arm1Status = "#4CAF50"; // green
    } else if (inputs.internalPositionRad < 0.872) {
      Arm1Status = "#F44336"; // red
    } else {
      Arm1Status = "#9528CC"; // purple
    }

    Logger.recordOutput("Arm1/angleInternalRad", inputs.internalPositionRad);
    Logger.recordOutput(
        "Arm1/angleInternalDeg", Units.radiansToDegrees(inputs.internalPositionRad));
    Logger.recordOutput("Arm1/motorVolts", inputs.appliedVolts);
    Logger.recordOutput("Arm1/Arm1IsUp", Arm1IsUp());
    Logger.recordOutput("Arm1/status", Arm1Status);

    // Update the Mechanism Arm angle based on the simulated arm angle
    Arm1.setAngle(Units.radiansToDegrees(inputs.internalPositionRad));
  }

  /** Set the goal for the arm angle. Put controller in auto if not already. */
  public void setGoalDeg(double setpointDeg) {
    Arm1ClosedLoop = true;
    angleGoalRad =
        Units.degreesToRadians(
            MathUtil.clamp(setpointDeg, Arm1Constants.ARM_POS_0, Arm1Constants.ARM_POS_1));
  }

  /** Stops the Arm. */
  public void stop() {
    io.stop();
    Arm1ClosedLoop = false;
  }

  /** Returns the current arm angle in degrees. */
  @AutoLogOutput(key = "Arm1/angleInternalDeg")
  public double getPositionDeg() {
    return Units.radiansToDegrees(inputs.internalPositionRad);
  }

  // Check if arm is within a tolerance of the down/load position.
  // Typically used as a permissive in commands.
  public boolean Arm1AtTarget() {
    return Units.radiansToDegrees(inputs.internalPositionRad)
        <= Arm1Constants.ARM_TARGET_DEG + Arm1Constants.ARM_IS_DOWN_TOLERANCE_DEG;
  }

  public boolean Arm1AtZero() {
    return Units.radiansToDegrees(inputs.internalPositionRad)
        <= Arm1Constants.ARM_IS_DOWN_TOLERANCE_DEG;
  }

  // Define "arm up" as an angle greater than 0 radians - currently no
  // shooting position would be lower than that
  public boolean Arm1IsUp() {
    return inputs.internalPositionRad >= 0.0;
  }

  // If the arm is less than 75 degrees it is not in an amp position
  public boolean Arm1IsDown() {
    return inputs.internalPositionRad < Units.degreesToRadians(-100.0);
  }

  // Has the arm reached the closed-loop goal?
  // Tolerance is set separately with ARM_ANGLE_TOLERANCE_DEG.
  public boolean atGoal() {
    return pid.atGoal();
  }

  // Move the arm to the stow/loading position.
  public Command Arm1ToTargetCommand() {
    return new RunCommand(() -> setGoalDeg(Arm1Constants.ARM_TARGET_DEG))
        .until(() -> Arm1AtTarget());
  }

  public Command Arm1ToZeroCommand() {
    return new RunCommand(() -> setGoalDeg(0.0)).until(() -> Arm1AtZero());
  }

  public Command Arm1ClimbCommand() {
    return new SequentialCommandGroup(
        Arm1ToTargetCommand()), 
        Arm1ToZeroCommand());
  }
}

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class PivotPositionCommand extends Command {
  private final Arm arm;

  private final Rotation2d pivotSetpoint;

  /**
   * Creates a new ScorePositionCommand.
   *
   * @param arm The arm subsystem to use in RobotContainer.
   * @param pivotSetpoint The pivot setpoint in Rotation2d.
   */
  public PivotPositionCommand(Arm arm, Rotation2d pivotSetpoint) {
    this.arm = arm;

    this.pivotSetpoint = pivotSetpoint;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setPivotTargetAngle(pivotSetpoint);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return arm.atPivotGoal();
  }
}

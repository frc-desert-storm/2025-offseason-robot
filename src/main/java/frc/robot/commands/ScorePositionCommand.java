package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ScorePositionCommand extends Command {
  private final Arm arm;

  private final Rotation2d pivotSetpoint;
  private final Rotation2d wristSetpoint;
  private final double extensionSetpoint;

  /**
   * Creates a new ScorePositionCommand.
   *
   * @param arm The arm subsystem to use in RobotContainer.
   * @param pivotSetpoint The pivot setpoint in Rotation2d.
   * @param wristSetpoint The wrist setpoint in Rotation2d.
   * @param extensionSetpointMeters The extension setpoint in meters.
   */
  public ScorePositionCommand(
      Arm arm, Rotation2d pivotSetpoint, Rotation2d wristSetpoint, double extensionSetpointMeters) {
    this.arm = arm;

    this.pivotSetpoint = pivotSetpoint;
    this.wristSetpoint = wristSetpoint;
    this.extensionSetpoint = extensionSetpointMeters;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setPivotTargetAngle(pivotSetpoint);
    arm.setWristTargetAngle(wristSetpoint);
    arm.setExtensionTargetLength(extensionSetpoint);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return arm.atGoal();
  }
}

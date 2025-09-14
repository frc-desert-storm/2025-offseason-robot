package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class WristPositionCommand extends Command {
  private final Arm arm;

  private final Rotation2d wristSetpoint;

  /**
   * Creates a new ScorePositionCommand.
   *
   * @param arm The arm subsystem to use in RobotContainer.
   * @param wristSetpoint The wrist setpoint in Rotation2d.
   */
  public WristPositionCommand(Arm arm, Rotation2d wristSetpoint) {
    this.arm = arm;

    this.wristSetpoint = wristSetpoint;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setWristTargetAngle(wristSetpoint);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return arm.atWristGoal();
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ExtensionPositionCommand extends Command {
  private final Arm arm;

  private final double extensionSetpoint;

  /**
   * Creates a new ScorePositionCommand.
   *
   * @param arm The arm subsystem to use.
   * @param extensionSetpointInches The extension setpoint in meters.
   */
  public ExtensionPositionCommand(Arm arm, double extensionSetpointInches) {
    this.arm = arm;

    this.extensionSetpoint = extensionSetpointInches;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setExtensionTargetLength(extensionSetpoint);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return arm.atExtensionGoal();
  }
}

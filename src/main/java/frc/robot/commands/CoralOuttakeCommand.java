package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class CoralOuttakeCommand extends Command {
  private final Coral coral;

  public CoralOuttakeCommand(Coral coral) {
    this.coral = coral;

    addRequirements(coral);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coral.setVoltage(6.0);
  }

  @Override
  public boolean isFinished() {
    return coral.getCurrent() > 4;
  }
}

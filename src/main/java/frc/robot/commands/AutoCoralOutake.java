package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class AutoCoralOutake extends Command {
  private final Coral coral;

  public AutoCoralOutake(Coral coral) {
    this.coral = coral;

    addRequirements(coral);
  }

  @Override
  public void execute() {
    coral.setVoltage(-5);
  }

  @Override
  public void initialize() {
    coral.setVoltage(-5);
  }
}

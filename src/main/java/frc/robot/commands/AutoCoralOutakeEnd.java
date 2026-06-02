package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class AutoCoralOutakeEnd extends Command {
  private final Coral coral;

  public AutoCoralOutakeEnd(Coral coral) {
    this.coral = coral;

    addRequirements(coral);
  }

  @Override
  public void initialize() {
    coral.setVoltage(0.5);
  }
}

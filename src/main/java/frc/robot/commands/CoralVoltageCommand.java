package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class CoralVoltageCommand extends Command {
  private final Coral coral;
  private final double volts;

  public CoralVoltageCommand(Coral coral, double volts) {
    this.coral = coral;
    this.volts = volts;

    addRequirements(coral);
  }

  @Override
  public void initialize() {
    coral.setVoltage(volts);
  }
}

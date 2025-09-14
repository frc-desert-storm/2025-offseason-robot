package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class CoralIntakeCommand extends Command {
  private final Coral coral;

  public CoralIntakeCommand(Coral coral) {
    this.coral = coral;

    addRequirements(coral);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coral.setVoltage(3);
  }

  @Override
  public void end(boolean idk) {
    coral.setVoltage(.5);
  }
}

// public class CoralIntakeCommand extends SequentialCommandGroup {
//   public CoralIntakeCommand(Coral coral) {
//     addCommands(
//         new CoralVoltageCommand(coral, 3.0),
//         new WaitCommand(0.1),
//         new WaitUntilCommand(() -> coral.getCurrent() > 40),
//         new CoralVoltageCommand(coral, 0.0));
//     addRequirements(coral);
//   }
// }

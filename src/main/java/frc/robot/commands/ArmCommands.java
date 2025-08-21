package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class ArmCommands {

  private ArmCommands() {}

  public static Command runPivot(Arm arm) {
    return Commands.run(
        () -> {
          arm.runPivot(Rotation2d.fromDegrees(20));
        },
        arm);
  }
}

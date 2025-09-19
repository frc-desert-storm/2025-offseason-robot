package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.drive.Drive;

public class CenterAuto extends SequentialCommandGroup {
  public CenterAuto(Drive drive, Arm arm, Coral coral) {
    addCommands(
        new WaitCommand(0.25),
        new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(22.059)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(139.082)),
                new ExtensionPositionCommand(arm, 0.0))
            .withTimeout(1),
        new AutoDrive(drive, 0.75, 0, 0).withTimeout(1.5),
        new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(22.059)).withTimeout(0.25),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(10)).withTimeout(0.5),
                new ExtensionPositionCommand(arm, 0.0).withTimeout(0.25))
            .withTimeout(1),
        new AutoDrive(drive, -0.75, 0, 0).withTimeout(1),
        new AutoDrive(drive, 0, 0.75, 0).withTimeout(0.125),
        new AutoDrive(drive, 0, 0, 0).withTimeout(0.1),
        new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(80)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(71.62)),
                new ExtensionPositionCommand(arm, 3.0))
            .withTimeout(1),
        new CoralOuttakeCommand(coral));
  }
}

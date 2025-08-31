package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.3;

  private DriveCommands() {}

  public static Command teleopDrive(
      Drive drive, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSpeed.getAsDouble(), DEADBAND);
          double y = MathUtil.applyDeadband(ySpeed.getAsDouble(), DEADBAND);
          double rot = MathUtil.applyDeadband(rotSpeed.getAsDouble(), DEADBAND);

          // Apply output
          drive.drive(x, y, rot, true);
        },
        drive);
  }

  public static InstantCommand resetPose(Drive drive) {
    return new InstantCommand(
        () -> {
          drive.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
        },
        drive);
  }

  public static Command auto(Drive drive) {
    return Commands.run(
            () -> {
              drive.drive(-1.0, 0.0, 0.0, true);
            })
        .withTimeout(5);
  }
}

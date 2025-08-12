package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  public static Command teleopDrive(
      Drive drive,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeed,
      DoubleSupplier period) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSpeed.getAsDouble(), DEADBAND);
          double y = MathUtil.applyDeadband(ySpeed.getAsDouble(), DEADBAND);
          double rot = MathUtil.applyDeadband(rotSpeed.getAsDouble(), DEADBAND);

          // Apply output
          drive.drive(x, y, rot, true, period.getAsDouble());
        },
        drive);
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoDrive extends Command {
  private final Drive drivetrain;
  private final double xSpeed;
  private final double ySpeed;
  private final double rot;

  public AutoDrive(Drive drive, double xSpeed, double ySpeed, double rot) {
    drivetrain = drive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    drivetrain.drive(xSpeed, ySpeed, rot, false);
  }
}

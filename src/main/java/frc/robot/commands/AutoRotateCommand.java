package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoRotateCommand extends Command {

  private final Drive drive;
  private final double rotSetpoint;
  //private final double ySpeed;
  private final double rotSpeed;

  public AutoRotateCommand(Drive drive, double rotSpeed, double rotInDegrees) {
    this.drive = drive;
    //this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    if(rotInDegrees < 0){
      rotSetpoint = drive.getPose() - rotInDegrees;
    } else {
      rotSetpoint = drive.getPose() + rotInDegrees;
    }
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.drive(0, 0, rotSpeed, false);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    if(rotSetpoint < drive.getPose()){
      return (rotSetpoint == (drive.getPose() + 0.5));
    } else{
      return (rotSetpoint == (drive.getPose() - 0.5));
    }
  }
}

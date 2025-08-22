package frc.robot.subsystems.arm;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.extension.ExtensionIO;
import frc.robot.subsystems.arm.extension.ExtensionIOInputsAutoLogged;
import frc.robot.subsystems.arm.pivot.PivotIO;
import frc.robot.subsystems.arm.pivot.PivotIOInputsAutoLogged;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ExtensionIO extensionIO;
  private final ExtensionIOInputsAutoLogged extensionInputs = new ExtensionIOInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
  private final WristIO wristIO;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  public Arm(ExtensionIO extensionIO, PivotIO pivotIO, WristIO wristIO) {
    this.extensionIO = extensionIO;
    this.pivotIO = pivotIO;
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    extensionIO.updateInputs(extensionInputs);
    pivotIO.updateInputs(pivotInputs);
    wristIO.updateInputs(wristInputs);
    Logger.processInputs("Arm/Extension", extensionInputs);
    Logger.processInputs("Arm/Pivot", pivotInputs);

    Logger.processInputs("Arm/Wrist", wristInputs);
  }

  public void setTargetAngle(Rotation2d target) {
    pivotIO.setTargetAngle(target);
  }

  public double getTargetAngle() {
    return pivotIO.getTargetAngle().getDegrees();
  }

  public void setWristTargetAngle(Rotation2d target) {
    wristIO.setTargetAngle(target);
  }

  public void resetPivot(Rotation2d target) {
    pivotIO.resetPosition(target);
  }

  public void setExtensionSetPoint(double setPointMeters){
    extensionIO.setTargetPosition(setPointMeters);
  }

  public double getExtensionSetPoint(){
    return extensionIO.getTargetPosition();
  }
  
  public void resetWrist(Rotation2d target) {
    wristIO.resetPosition(target);
  }
}

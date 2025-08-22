package frc.robot.subsystems.arm;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.extension.ExtensionIO;
import frc.robot.subsystems.arm.extension.ExtensionIOInputsAutoLogged;
import frc.robot.subsystems.arm.pivot.PivotIO;
import frc.robot.subsystems.arm.pivot.PivotIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ExtensionIO extensionIO;
  private final ExtensionIOInputsAutoLogged extensionInputs = new ExtensionIOInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  public Arm(ExtensionIO extensionIO, PivotIO pivotIO) {
    this.extensionIO = extensionIO;
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    extensionIO.updateInputs(extensionInputs);
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Arm/Extension", extensionInputs);
    Logger.processInputs("Arm/Pivot", pivotInputs);
  }

  public void setTargetAngle(Rotation2d target) {
    pivotIO.setTargetAngle(target);
  }

  public double getTargetAngle() {
    return pivotIO.getTargetAngle().getDegrees();
  }

  public void resetPivot(Rotation2d target) {
    pivotIO.resetPosition(target);
  }
}

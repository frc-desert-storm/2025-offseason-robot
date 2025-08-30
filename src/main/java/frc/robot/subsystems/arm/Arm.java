package frc.robot.subsystems.arm;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  private final Mechanism2d mechanism = new Mechanism2d(1.0, 1.0);

  private final MechanismLigament2d mechanismArm =
      new MechanismLigament2d("arm", Units.inchesToMeters(29), 90, 6, new Color8Bit(Color.kPurple));

  private final MechanismLigament2d mechanismWrist =
      new MechanismLigament2d("wrist", Units.inchesToMeters(5), 90, 6, new Color8Bit(Color.kBlue));

  public Arm(ExtensionIO extensionIO, PivotIO pivotIO, WristIO wristIO) {
    this.extensionIO = extensionIO;
    this.pivotIO = pivotIO;
    this.wristIO = wristIO;

    MechanismRoot2d mechanismRoot = mechanism.getRoot("root", 0, 0);
    mechanismRoot.append(mechanismArm);
    mechanismArm.append(mechanismWrist);
  }

  @Override
  public void periodic() {
    extensionIO.updateInputs(extensionInputs);
    pivotIO.updateInputs(pivotInputs);
    wristIO.updateInputs(wristInputs);

    Logger.processInputs("Arm/Extension", extensionInputs);
    Logger.processInputs("Arm/Pivot", pivotInputs);
    Logger.processInputs("Arm/Wrist", wristInputs);

    updateMechanism();
    SmartDashboard.putData("Arm/Mech2d", mechanism);
  }

  public void updateMechanism() {
    mechanismArm.setAngle(
        Rotation2d.fromRadians(Units.radiansToDegrees(pivotInputs.pivotLeftPositionRad)));
    mechanismWrist.setAngle(
        Rotation2d.fromRadians(Units.radiansToDegrees(wristInputs.wristPositionRad)));
    mechanismArm.setLength(extensionInputs.extensionPositionMeters + Units.inchesToMeters(29));
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

  public void setExtensionSetPoint(double setPointMeters) {
    extensionIO.setTargetPosition(setPointMeters);
  }

  public double getExtensionSetPoint() {
    return extensionIO.getTargetPosition();
  }

  public void resetExtension(double target) {
    extensionIO.resetPosition(target);
  }

  public void resetWrist(Rotation2d target) {
    wristIO.resetPosition(target);
  }
}

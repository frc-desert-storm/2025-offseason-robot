package frc.robot.subsystems.arm;

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

  /** Updates the mechanism2d on the dashboard. */
  public void updateMechanism() {
    mechanismArm.setAngle(Rotation2d.fromRadians(pivotInputs.pivotLeftPositionRad));
    mechanismWrist.setAngle(Rotation2d.fromRadians(wristInputs.wristPositionRad));
    mechanismArm.setLength(extensionInputs.extensionPositionInches + Units.inchesToMeters(29));
  }

  /** Sets the pivot target angle. */
  public void setPivotTargetAngle(Rotation2d target) {
    pivotIO.setTargetAngle(target);
  }

  /** Gets the pivot target angle. */
  public Rotation2d getPivotTargetAngle() {
    return pivotIO.getTargetAngle();
  }

  /** Resets the pivot to the target angle. */
  public void resetPivot(Rotation2d target) {
    pivotIO.resetPosition(target);
  }

  /** Sets the wrist target angle. */
  public void setWristTargetAngle(Rotation2d target) {
    wristIO.setTargetAngle(target);
  }

  /** Gets the wrist target angle. */
  public Rotation2d getWristTargetAngle() {
    return wristIO.getTargetAngle();
  }

  /** Resets the wrist to the target angle. */
  public void resetWrist(Rotation2d target) {
    wristIO.resetPosition(target);
  }

  /**
   * sets the extension target position.
   *
   * @param setPointMeters the target position in meters
   */
  public void setExtensionTargetLength(double setPointMeters) {
    extensionIO.setTargetPosition(setPointMeters);
  }

  /**
   * Gets the extension target position.
   *
   * @return the target position in meters
   */
  public double getExtensionTargetPosition() {
    return extensionIO.getTargetPosition();
  }

  /**
   * Resets the extension to the target position.
   *
   * @param target the target position in meters
   */
  public void resetExtension(double target) {
    extensionIO.resetPosition(target);
  }

  /** Returns true if the wrist, extension and pivot are at their goal. */
  public boolean atGoal() {
    return extensionIO.atGoal() && pivotIO.atGoal() && wristIO.atGoal();
  }
}

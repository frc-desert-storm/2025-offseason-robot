package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ExtensionPositionCommand;
import frc.robot.commands.PivotPositionCommand;
import frc.robot.commands.WristPositionCommand;
import frc.robot.subsystems.arm.Arm;

enum ScoringConstants {
  L1(Rotation2d.fromDegrees(20), Rotation2d.fromRadians(140), 0.0, true),
  L2(Rotation2d.fromDegrees(58), Rotation2d.fromRadians(257), 0.0, true),
  L3(Rotation2d.fromDegrees(80), Rotation2d.fromRadians(70), 3.0, false),
  L3Front(),
  Intake(Rotation2d.fromDegrees(52.5), Rotation2d.fromRadians(3.56), 0.0, true),
  GroundIntake(Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(190), 0.0, true),
  Stow(Rotation2d.fromDegrees(25), Rotation2d.fromDegrees(70), 0.0, true);

  private final Rotation2d wristAngle;
  private final Rotation2d pivotAngle;
  private final double extensionPosition;
  private final boolean extensionFirst;

  ScoringConstants(
      Rotation2d pivotAngle,
      Rotation2d wristAngle,
      double extensionPosition,
      boolean extensionFirst) {
    this.pivotAngle = pivotAngle;
    this.wristAngle = wristAngle;
    this.extensionPosition = extensionPosition;
    this.extensionFirst = extensionFirst;
  }

  ScoringConstants() {
    this(null, null, 0.0, false);
  } // TODO remove this once values are set

  public SequentialCommandGroup getCommand(Arm arm) {
    if (extensionFirst) {
      return new SequentialCommandGroup(
          new ExtensionPositionCommand(arm, extensionPosition),
          new PivotPositionCommand(arm, pivotAngle),
          new WristPositionCommand(arm, wristAngle));
    } else {
      return new SequentialCommandGroup(
          new PivotPositionCommand(arm, pivotAngle),
          new WristPositionCommand(arm, wristAngle),
          new ExtensionPositionCommand(arm, extensionPosition));
    }
  }
}

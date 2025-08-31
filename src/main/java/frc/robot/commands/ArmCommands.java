package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;

public class ArmCommands {
  private static final double DEADBAND = 0.3;

  private ArmCommands() {}

  public static Command intake(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          arm.setPivotTargetAngle(Rotation2d.fromDegrees(45));
        },
        arm);
  }

  public static Command score(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          arm.setPivotTargetAngle(Rotation2d.fromDegrees(90));
        },
        arm);
  }

  public static Command moveArmUp(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          double setpoint = arm.getPivotTargetAngle().getDegrees();
          Logger.recordOutput("arm/pivot", setpoint);
          if (setpoint <= 120) {
            arm.setPivotTargetAngle(Rotation2d.fromDegrees(setpoint + 0.5));
          }
        },
        arm);
  }

  public static Command moveArmDown(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          double setpoint = arm.getPivotTargetAngle().getDegrees();
          Logger.recordOutput("arm/pivot", setpoint);
          if (setpoint >= -9.8) {
            arm.setPivotTargetAngle(Rotation2d.fromDegrees(setpoint - 0.5));
          }
        },
        arm);
  }

  public static Command scoreCommand(Arm arm) {
    return Commands.run(
        () -> {
          arm.setWristTargetAngle(Rotation2d.fromDegrees(100));
        },
        arm);
  }

  public static Command intakeCommand(Arm arm) {
    return Commands.run(
        () -> {
          arm.setWristTargetAngle(Rotation2d.fromDegrees(-90));
        },
        arm);
  }

  public static Command resetArmPose(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          arm.resetPivot(Rotation2d.fromDegrees(-10));
          arm.resetWrist(Rotation2d.fromDegrees(-10));
          arm.resetExtension(0);
        },
        arm);
  }

  public static Command testArm(Arm arm) {
    return Commands.run(
        () -> {
          arm.setExtensionTargetPosition(Units.inchesToMeters(8));
        },
        arm);
  }

  public static Command extendArm(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          double setpoint = arm.getExtensionTargetPosition();
          if (Units.metersToInches(setpoint) <= 23.3) {
            arm.setExtensionTargetPosition(setpoint + Units.inchesToMeters(0.2));
          }
        },
        arm);
  }

  public static Command retractArm(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          double setpoint = arm.getExtensionTargetPosition();
          if (Units.metersToInches(setpoint) >= 0.2) {
            arm.setExtensionTargetPosition(setpoint - Units.inchesToMeters(0.2));
          }
        },
        arm);
  }
}

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class ArmCommands {
  private static final double DEADBAND = 0.3;

  private ArmCommands() {}

  public static Command intake(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          arm.setTargetAngle(Rotation2d.fromDegrees(45));
        },
        arm);
  }

  public static Command score(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          arm.setTargetAngle(Rotation2d.fromDegrees(90));
        },
        arm);
  }

  public static Command moveArmUp(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          double setpoint = arm.getTargetAngle();
          if (setpoint <= 118) arm.setTargetAngle(Rotation2d.fromDegrees(setpoint += 0.2));
        },
        arm);
  }

  public static Command moveArmDown(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          double setpoint = arm.getTargetAngle();
          if (setpoint >= -8) arm.setTargetAngle(Rotation2d.fromDegrees(setpoint -= 0.2));
        },
        arm);
  }

  public static Command resetPivot(Arm arm) {
    return Commands.run(
        () -> {
          // Apply output
          arm.resetPivot(Rotation2d.fromDegrees(-10));
          ;
        },
        arm);
  }
}

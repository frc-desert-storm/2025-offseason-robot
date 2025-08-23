// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeed;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.extension.ExtensionIO;
import frc.robot.subsystems.arm.extension.ExtensionIOSparkMax;
import frc.robot.subsystems.arm.pivot.PivotIO;
import frc.robot.subsystems.arm.pivot.PivotIOSparkMax;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIOSparkMax;
import frc.robot.subsystems.drive.*;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(5);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOSparkMax(), new GyroIOPigeon2());
        arm = new Arm(new ExtensionIOSparkMax(), new PivotIOSparkMax(), new WristIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSparkMax(), new GyroIO() {});
        arm = new Arm(new ExtensionIOSparkMax(), new PivotIOSparkMax(), new WristIOSparkMax());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        arm = new Arm(new ExtensionIO() {}, new PivotIO() {}, new WristIO() {});
        break;
    }

    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.teleopDrive(
            drive,
            () -> -xSpeedLimiter.calculate(driverController.getLeftY()) * kMaxLinearSpeed,
            () -> -ySpeedLimiter.calculate(driverController.getLeftX()) * kMaxLinearSpeed,
            () -> -rotSpeedLimiter.calculate(driverController.getRightX()) * kMaxAngularSpeed));
    driverController.start().onTrue(DriveCommands.resetPose(drive));

    driverController.a().onTrue(ArmCommands.intake(arm));
    driverController.b().onTrue(ArmCommands.resetArmPose(arm));
    driverController.x().onTrue(ArmCommands.score(arm));
    driverController.y().onTrue(ArmCommands.testWrist(arm));
    // driverController.leftBumper().onTrue(ArmCommands.testArm(arm));
    driverController.rightBumper().whileTrue(ArmCommands.moveArmUp(arm));
    driverController.leftBumper().whileTrue(ArmCommands.moveArmDown(arm));
    // driverController.rightTrigger(0.2).onTrue(ArmCommands.extendArm(arm));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void teleopInit() {
    CommandScheduler.getInstance().schedule(ArmCommands.resetArmPose(arm));
  }
}

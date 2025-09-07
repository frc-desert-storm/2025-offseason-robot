// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeed;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScorePositionCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.extension.ExtensionIO;
import frc.robot.subsystems.arm.extension.ExtensionIOSim;
import frc.robot.subsystems.arm.extension.ExtensionIOSparkMax;
import frc.robot.subsystems.arm.pivot.PivotIO;
import frc.robot.subsystems.arm.pivot.PivotIOSim;
import frc.robot.subsystems.arm.pivot.PivotIOSparkMax;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIOSim;
import frc.robot.subsystems.arm.wrist.WristIOSparkMax;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(5);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOSparkMax(), new GyroIOPigeon2());
        arm = new Arm(new ExtensionIOSparkMax(), new PivotIOSparkMax(), new WristIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        arm = new Arm(new ExtensionIOSim(), new PivotIOSim(), new WristIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        arm = new Arm(new ExtensionIO() {}, new PivotIO() {}, new WristIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("simple", DriveCommands.auto(drive));

    // Configure the button bindings
    configureBindings();
  }

  /** Configure the button bindings. */
  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.teleopDrive(
            drive,
            () -> -xSpeedLimiter.calculate(driverController.getLeftY()) * kMaxLinearSpeed,
            () -> -ySpeedLimiter.calculate(driverController.getLeftX()) * kMaxLinearSpeed,
            () -> -rotSpeedLimiter.calculate(driverController.getRightX()) * kMaxAngularSpeed));
    driverController.start().onTrue(DriveCommands.resetPose(drive));

    // driverController.a().onTrue(ArmCommands.intake(arm));
    // driverController.b().onTrue(ArmCommands.resetArmPose(arm));
    // driverController.x().onTrue(ArmCommands.score(arm));
    // driverController.y().whileTrue(ArmCommands.extendArm(arm));

    // driverController.leftBumper().onTrue(ArmCommands.scoreCommand(arm));
    // driverController.rightBumper().onTrue(ArmCommands.intakeCommand(arm));
    // driverController.rightTrigger(.1).whileTrue(ArmCommands.moveArmUp(arm));
    // driverController.leftTrigger(.1).whileTrue(ArmCommands.moveArmDown(arm));
    // driverController.rightTrigger(0.2).onTrue(ArmCommands.extendArm(arm));

    driverController
        .a()
        .onTrue(new ScorePositionCommand(arm, Rotation2d.fromDegrees(90), Rotation2d.kZero, 0.0));
    driverController
        .b()
        .onTrue(new ScorePositionCommand(arm, Rotation2d.fromDegrees(45), Rotation2d.kZero, 0.0));
    driverController
        .x()
        .onTrue(new ScorePositionCommand(arm, Rotation2d.fromDegrees(0), Rotation2d.kZero, 0.0));
    driverController
        .y()
        .onTrue(new ScorePositionCommand(arm, Rotation2d.fromDegrees(60), Rotation2d.kZero, 0.0));
    driverController
        .rightBumper()
        .onTrue(
            new ScorePositionCommand(
                arm,
                Rotation2d.fromDegrees(0),
                Rotation2d.kZero,
                0.1)); // Move pivot to 0 degrees, and extend by 10cm
    driverController
        .leftBumper()
        .onTrue(
            new ScorePositionCommand(
                arm,
                Rotation2d.fromDegrees(0),
                Rotation2d.kZero,
                0)); // Move pivot to 0 degrees, and go to 0cm
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeed;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
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
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOSparkMax;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Coral coral;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.operatorControllerPort);

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
        coral = new Coral(new CoralIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        arm = new Arm(new ExtensionIOSim(), new PivotIOSim(), new WristIOSim());
        coral = new Coral(new CoralIOSparkMax());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        arm = new Arm(new ExtensionIO() {}, new PivotIO() {}, new WristIO() {});
        coral = new Coral(new CoralIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("simple", DriveCommands.auto(drive));

    // Pathplanner Named Commands
    NamedCommands.registerCommand(
        "pivot to L3", new PivotPositionCommand(arm, Rotation2d.fromDegrees(80)));
    NamedCommands.registerCommand(
        "pivot to Intake", new PivotPositionCommand(arm, Rotation2d.fromDegrees(52.5)));
    NamedCommands.registerCommand(
        "pivot to L2", new PivotPositionCommand(arm, Rotation2d.fromDegrees(58)));
    NamedCommands.registerCommand(
        "de-algae pivot", new PivotPositionCommand(arm, Rotation2d.fromDegrees(22.059)));

    NamedCommands.registerCommand(
        "wrist to L3", new WristPositionCommand(arm, Rotation2d.fromDegrees(71.62)));
    NamedCommands.registerCommand(
        "de-algae wrist 1", new WristPositionCommand(arm, Rotation2d.fromDegrees(139.082)));
    NamedCommands.registerCommand(
        "de-algae wrist 2", new WristPositionCommand(arm, Rotation2d.fromDegrees(10)));
    NamedCommands.registerCommand(
        "wrist to L2", new WristPositionCommand(arm, Rotation2d.fromDegrees(257)));

    NamedCommands.registerCommand("extension to L3", new ExtensionPositionCommand(arm, 3.0));
    NamedCommands.registerCommand("extension to L2", new ExtensionPositionCommand(arm, 0.0));

    NamedCommands.registerCommand("score", new AutoCoralOutake(coral));
    NamedCommands.registerCommand("end", new AutoCoralOutakeEnd(coral));

    autoChooser.addOption("move", new CenterAuto(drive, arm, coral));

    // Configure the button bindings
    configureBindings();
  }

  /** Configure the button bindings. */
  private void configureBindings() {
    // Drive, linear movement with Left Joystick
    // Drive, rotation with Right Joystick
    drive.setDefaultCommand(
        DriveCommands.teleopDrive(
            drive,
            () -> -xSpeedLimiter.calculate(driverController.getLeftY()) * kMaxLinearSpeed,
            () -> -ySpeedLimiter.calculate(driverController.getLeftX()) * kMaxLinearSpeed,
            () -> -rotSpeedLimiter.calculate(driverController.getRightX()) * kMaxAngularSpeed));

    // Reset Gyro
    driverController.start().onTrue(DriveCommands.resetPose(drive));

    // Move to intaking position, and spin wheels to intake
    // HOLD DOWN LEFT TRIGGER TO INTAKE
    driverController
        .leftTrigger()
        .onTrue(
            new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(52.5)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(203.973)),
                new ExtensionPositionCommand(arm, 0.0)));

    // Move to score position for L3
    // DOES NOT SCORE, ONLY MOVES TO POSITION
    driverController
        .x()
        .onTrue(
            new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(80)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(71.62)),
                new ExtensionPositionCommand(arm, 3.0)));

    // Move to score position for L2
    // DOES NOT SCORE, ONLY MOVES TO POSITION
    driverController
        .b()
        .onTrue(
            new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(58)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(257)),
                new ExtensionPositionCommand(arm, 0.0)));

    // Move to remove algae on L2
    // Must release button to actually "remove algae" STILL IN TESTING
    driverController
        .y()
        .onTrue(
            new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(22.059)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(139.082)),
                new ExtensionPositionCommand(arm, 0.0)))
        .onFalse(
            new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(22.059)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(10)),
                new ExtensionPositionCommand(arm, 0.0)));

    // Move to score on L1
    // DOES NOT SCORE, ONLY MOVES TO POSITION. STILL IN TESTING
    driverController
        .a()
        .onTrue(
            new SequentialCommandGroup(
                new PivotPositionCommand(arm, Rotation2d.fromDegrees(20)),
                new WristPositionCommand(arm, Rotation2d.fromDegrees(140)),
                new ExtensionPositionCommand(arm, 0.0)));

    // See what happens w/ out this because why do we have the same command twice?

    // Spins manipulator wheels for scoring
    driverController.rightTrigger().whileTrue(new CoralOuttakeCommand(coral));

    // Spins manipulator wheels for intaking
    driverController.leftTrigger().whileTrue(new CoralIntakeCommand(coral));

    // OPERATOR CONTROLS
    operatorController.rightTrigger(.1).whileTrue(ArmCommands.moveArmUp(arm));
    operatorController.leftTrigger(.1).whileTrue(ArmCommands.moveArmDown(arm));

    operatorController.leftBumper().whileTrue(ArmCommands.retractArm(arm));
    operatorController.rightBumper().whileTrue(ArmCommands.extendArm(arm));

    // Previous Controls

    /*driverController.a().onTrue(ArmCommands.intake(arm));
    driverController.b().onTrue(ArmCommands.resetArmPose(arm));
    driverController.x().onTrue(ArmCommands.score(arm));
    driverController.y().whileTrue(ArmCommands.extendArm(arm));

    driverController.leftBumper().onTrue(ArmCommands.scoreCommand(arm));
    driverController.rightBumper().onTrue(ArmCommands.intakeCommand(arm));
    driverController.rightTrigger(.1).whileTrue(ArmCommands.moveArmUp(arm));
    driverController.leftTrigger(.1).whileTrue(ArmCommands.moveArmDown(arm));
    driverController.rightTrigger(0.2).onTrue(ArmCommands.extendArm(arm));

    driverController
        .a()
        .whileTrue(
            new SequentialCommandGroup(
                    new ExtensionPositionCommand(arm, 0.0),
                    new PivotPositionCommand(arm, Rotation2d.fromDegrees(50)),
                    new WristPositionCommand(arm, Rotation2d.fromDegrees(315)))
                .andThen(
                    new ExtensionPositionCommand(arm, 0.0),
                    new PivotPositionCommand(arm, Rotation2d.fromDegrees(40)),
                    new WristPositionCommand(arm, Rotation2d.fromDegrees(45))));

    driverController
        .b()
        .onTrue(
            new SequentialCommandGroup(new PivotPositionCommand(arm, Rotation2d.fromDegrees(45))));
    driverController
        .x()
        .onTrue(
            new SequentialCommandGroup(new WristPositionCommand(arm, Rotation2d.fromDegrees(45))));
    driverController
         .y()
         .onTrue(
           new SequentialCommandGroup(new ExtensionPositionCommand(arm,12)));
     */

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

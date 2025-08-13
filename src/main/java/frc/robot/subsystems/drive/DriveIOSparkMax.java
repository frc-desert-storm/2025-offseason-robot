// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class DriveIOSparkMax implements DriveIO {

  private final SparkMax frontLeftMotor =
      new SparkMax(frontLeftCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax frontRightMotor =
      new SparkMax(frontRightCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax backLeftMotor =
      new SparkMax(backLeftCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax backRightMotor =
      new SparkMax(backRightCanId, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private final RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
  private final RelativeEncoder backLeftEncoder = backLeftMotor.getEncoder();
  private final RelativeEncoder backRightEncoder = backRightMotor.getEncoder();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(realKs, realKv);

  private final PIDController frontLeftPIDController = new PIDController(realKp, 0, realKd);
  private final PIDController frontRightPIDController = new PIDController(realKp, 0, realKd);
  private final PIDController backLeftPIDController = new PIDController(realKp, 0, realKd);
  private final PIDController backRightPIDController = new PIDController(realKp, 0, realKd);

  public DriveIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(2 * Math.PI / motorReduction) // Rotor Rotations -> Wheel Radians
        .velocityConversionFactor(
            (2 * Math.PI) / 60.0 / motorReduction) // Rotor RPM -> Wheel Rad/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    config.inverted(frontLeftInverted);
    frontLeftMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(frontRightInverted);
    frontRightMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(backLeftInverted);
    backLeftMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(backRightInverted);
    backRightMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */
  @Override
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        frontLeftPIDController.calculate(
            frontLeftEncoder.getVelocity(), speeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        frontRightPIDController.calculate(
            frontRightEncoder.getVelocity(), speeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        backLeftPIDController.calculate(
            backLeftEncoder.getVelocity(), speeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        backRightPIDController.calculate(
            backRightEncoder.getVelocity(), speeds.rearRightMetersPerSecond);

    frontLeftMotor.setVoltage((frontLeftOutput + frontLeftFeedforward));
    frontRightMotor.setVoltage((frontRightOutput + frontRightFeedforward));
    backLeftMotor.setVoltage((backLeftOutput + backLeftFeedforward));
    backRightMotor.setVoltage((backRightOutput + backRightFeedforward));
  }

  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        frontLeftEncoder.getVelocity(),
        frontRightEncoder.getVelocity(),
        backLeftEncoder.getVelocity(),
        backRightEncoder.getVelocity());
  }

  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        frontLeftEncoder.getPosition(),
        frontRightEncoder.getPosition(),
        backLeftEncoder.getPosition(),
        backRightEncoder.getPosition());
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // front left motor
    inputs.frontLeftPositionRad = frontLeftEncoder.getPosition();
    inputs.frontLeftVelocityRadPerSec = frontLeftEncoder.getVelocity();
    inputs.frontLeftAppliedVolts =
        frontLeftMotor.getAppliedOutput() * frontLeftMotor.getBusVoltage();
    inputs.frontLeftCurrentAmps = frontLeftMotor.getOutputCurrent();

    // front right motor
    inputs.frontRightPositionRad = frontRightEncoder.getPosition();
    inputs.frontRightVelocityRadPerSec = frontRightEncoder.getVelocity();
    inputs.frontRightAppliedVolts =
        frontRightMotor.getAppliedOutput() * frontRightMotor.getBusVoltage();
    inputs.frontRightCurrentAmps = frontRightMotor.getOutputCurrent();

    // back left motor
    inputs.backLeftPositionRad = backLeftEncoder.getPosition();
    inputs.backLeftVelocityRadPerSec = backLeftEncoder.getVelocity();
    inputs.backLeftAppliedVolts = backLeftMotor.getAppliedOutput() * backLeftMotor.getBusVoltage();
    inputs.backLeftCurrentAmps = backLeftMotor.getOutputCurrent();

    // back right motor
    inputs.backRightPositionRad = backRightEncoder.getPosition();
    inputs.backRightVelocityRadPerSec = backRightEncoder.getVelocity();
    inputs.backRightAppliedVolts =
        backRightMotor.getAppliedOutput() * backRightMotor.getBusVoltage();
    inputs.backRightCurrentAmps = backRightMotor.getOutputCurrent();
  }
}

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

import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
  public static final double kMaxLinearSpeed = 3.0;
  public static final double kMaxAngularSpeed = Math.PI;

  public static final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  public static final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  public static final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  public static final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  // Device CAN IDs
  public static final int pigeonCanId = 10;
  public static final int frontLeftCanId = 11;
  public static final int frontRightCanId = 12;
  public static final int backLeftCanId = 13;
  public static final int backRightCanId = 14;

  // Motor configuration
  public static final int currentLimit = 50;
  public static final double motorReduction = 8.45;
  public static final boolean frontLeftInverted = false;
  public static final boolean frontRightInverted = true;
  public static final boolean backLeftInverted = false;
  public static final boolean backRightInverted = true;

  // Velocity PID configuration
  public static final double realKp = 0.001;
  public static final double realKd = 0.0;
  public static final double realKs = 1.0;
  public static final double realKv = 1.0;

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;

  //  // PathPlanner configuration
  // public static final double wheelRadiusMeters = Units.inchesToMeters(3.0);
  // public static final DCMotor gearbox = DCMotor.getNEO(1);
  //  public static final double trackWidth = (frontLeftLocation.getX() + backLeftLocation.getX()) /
  // 2.0 + (frontRightLocation.getX() + backRightLocation.getX()) / 2.0; // Get average trackWidth
  //  public static final double robotMassKg = 74.088;
  //  public static final double robotMOI = 6.883;
  //  public static final double wheelCOF = 1.2;
  //  public static final RobotConfig ppConfig =
  //      new RobotConfig(
  //          robotMassKg,
  //          robotMOI,
  //          new ModuleConfig(
  //              wheelRadiusMeters,
  //              maxSpeedMetersPerSec,
  //              wheelCOF,
  //              gearbox.withReduction(motorReduction),
  //              currentLimit,
  //              4),
  //              frontLeftLocation.getX() + frontRightLocation.getX());
}

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

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double frontLeftPositionRad = 0.0;
    public double frontLeftVelocityRadPerSec = 0.0;
    public double frontLeftAppliedVolts = 0.0;
    public double frontLeftCurrentAmps = 0.0;

    public double frontRightPositionRad = 0.0;
    public double frontRightVelocityRadPerSec = 0.0;
    public double frontRightAppliedVolts = 0.0;
    public double frontRightCurrentAmps = 0.0;

    public double backLeftPositionRad = 0.0;
    public double backLeftVelocityRadPerSec = 0.0;
    public double backLeftAppliedVolts = 0.0;
    public double backLeftCurrentAmps = 0.0;

    public double backRightPositionRad = 0.0;
    public double backRightVelocityRadPerSec = 0.0;
    public double backRightAppliedVolts = 0.0;
    public double backRightCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run the specified speeds. */
  public default void setSpeeds(MecanumDriveWheelSpeeds speeds) {
     DriveIOSparkMax max = new DriveIOSparkMax();
     max.setSpeeds(speeds);
  }

  public default MecanumDriveWheelSpeeds getCurrentState() {
    return null;
  }

  public default MecanumDriveWheelPositions getCurrentDistances() {
    return null;
  }
}

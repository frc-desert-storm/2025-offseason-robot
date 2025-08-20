// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  
  public static final double BROWNOUT_VOLTAGE_LIMIT = 7.5;

  public static final int FRONT_L_MOTOR_CHANNEL = 1;
  public static final int FRONT_R_MOTOR_CHANNEL = 2;
  public static final int BACK_L_MOTOR_CHANNEL = 3;
  public static final int BACK_R_MOTOR_CHANNEL = 4;

  // Placeholders
  public static final int ARM_BASE_L_MOTOR_CHANNEL = -1;
  public static final int ARM_BASE_R_MOTOR_CHANNEL = -1;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}

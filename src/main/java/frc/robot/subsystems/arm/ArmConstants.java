package frc.robot.subsystems.arm;

public class ArmConstants {

  // PID Values
  public static final double realKp = 0.001;
  public static final double realKd = 0.0;
  public static final double realKs = 1.0;
  public static final double realKv = 1.0;

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;

  // Motor Configuration
  public static final int currentLimit = 50;

  public static final boolean basePivotLeftInverted = true;
  public static final boolean basePivotRightInverted = false;
  public static final boolean armExtenderInverted = false;
  public static final boolean manipulatorInverted = false;

  public static double basepivotReduction = 1;
  public static double extenderReduction = 1;
  public static double manipulatorReduction = 1;

  // CAN ID's
  public static final int basePivotLeftCanId = 21;
  public static final int basePivotRightCanID = 22;
  public static final int armExtenderCanID = 23;
  public static final int manipulatorCanId = 24;
}

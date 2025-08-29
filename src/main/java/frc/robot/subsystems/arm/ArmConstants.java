package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {

  // Pivot
  public static final int pivotLeftCanId = 21;
  public static final int pivotRightCanId = 22;

  public static final boolean pivotLeftInverted = false;
  public static final boolean pivotRightInverted = true;

  public static final int pivotReduction = 36;
  public static final int pivotCurrentLimit = 60;

  public static final double pivotMaxAccel = 200.0;
  public static final double pivotMaxVelo = 400.0;

  public static final double pivotRealKp = 1.0;
  public static final double pivotRealKd = 0.0;

  public static final double pivotRealKv = 0.0;
  public static final double pivotRealKg = 0.8;
  public static final double pivotRealKs = 0.0;
  public static final double pivotRealKa = 0.0;

  public static final double pivotSimKp = 1.0;
  public static final double pivotSimKd = 0.0;

  public static final double pivotSimKv = 0.0;
  public static final double pivotSimKg = 1.0;
  public static final double pivotSimKs = 0.0;
  public static final double pivotSimKa = 0.0;
  // Extension
  public static final int extensionCanId = 23;

  public static final boolean extensionInverted = false;

  public static final double extensionReduction = Units.inchesToMeters(2.8125);
  public static final int extensionCurrentLimit = 30;

  public static final double extensionMaxAccel = 200.0;
  public static final double extensionMaxVelo = 400.0;

  public static final double extensionRealKp = 1.0;
  public static final double extensionRealKd = 0.0;

  public static final double extensionRealKv = 0.0;
  public static final double extensionRealKs = 0.0;
  public static final double extensionRealKa = 0.0;

  public static final double extensionSimKp = 1.0;
  public static final double extensionSimKd = 0.0;

  public static final double extensionSimKv = 0.0;
  public static final double extensionSimKs = 0.0;
  public static final double extensionSimKa = 0.0;

  // Wrist
  public static final int wristCanId = 24;

  public static final boolean wristInverted = false;

  public static final int wristReduction = 4;
  public static final int wristCurrentLimit = 60;

  public static final double wristMaxAccel = 200.0;
  public static final double wristMaxVelo = 400.0;

  public static final double wristRealKp = 1.0;
  public static final double wristRealKd = 0.0;

  public static final double wristRealKv = 0.0;
  public static final double wristRealKg = 0.8;
  public static final double wristRealKs = 0.0;
  public static final double wristRealKa = 0.0;

  public static final double wristSimKp = 1.0;
  public static final double wristSimKd = 0.0;

  public static final double wristSimKv = 0.0;
  public static final double wristSimKg = 1.0;
  public static final double wristSimKs = 0.0;
  public static final double wristSimKa = 0.0;
}

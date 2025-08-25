package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
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

  public static final DCMotor pivotGearbox = DCMotor.getNEO(2).withReduction(pivotReduction);

  // Extension
  public static final int extensionCanId = 23;

  public static final boolean extensionInverted = false;

  public static final double extensionReduction = Units.inchesToMeters(2.8125);
  public static final int extensionCurrentLimit = 60;
  public static final DCMotor extensionGearbox =
      DCMotor.getNEO(1).withReduction(extensionReduction);

  // Wrist
  public static final int wristCanId = 24;

  public static final boolean wristInverted = false;

  public static final int wristReduction = 4;
  public static final int wristCurrentLimit = 60;
}

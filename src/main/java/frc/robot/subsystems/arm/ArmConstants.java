package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {

  // Pivot
  public static final int pivotLeftCanId = 21;
  public static final int pivotRightCanId = 22;

  public static final boolean pivotLeftInverted = false;
  public static final boolean pivotRightInverted = true;

  public static final int pivotReduction = 36;
  public static final int pivotCurrentLimit = 40;
  public static final double pivotKS = 0.0;
  public static final double pivotKG = 0.0;
  public static final double pivotKV = 0.01;
  public static final double pivotKA = 0.0;

  public static final double pivotKP = 0.1;
  public static final double pivotKI = 0.0;
  public static final double pivotKD = 0.0;

  // Extension
  public static final int extensionCanId = 23;

  public static final boolean extensionInverted = false;

  public static final int extensionReduction = 36;
  public static final int extensionCurrentLimit = 60;
  public static final DCMotor extensionGearbox =
      DCMotor.getNEO(1).withReduction(extensionReduction);
}

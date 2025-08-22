package frc.robot.subsystems.arm;

public class ArmConstants {

  // TODO tune all values in this file!

  // Pivot
  public static final int pivotLeftCanId = 21;
  public static final int pivotRightCanId = 22;

  public static final boolean pivotLeftInverted = false;
  public static final boolean pivotRightInverted = true;

  public static final int pivotReduction = 36;
  public static final int pivotCurrentLimit = 40;
  public static final double pivotKS = 0.0;
  public static final double pivotKG = 2.07;
  public static final double pivotKV = 0.70;
  public static final double pivotKA = 0.83;

  public static final double pivotKP = 1.08;
  public static final double pivotKI = 0.0;
  public static final double pivotKD = 1.50;

  // Extension
  public static final int extensionCanId = 23;

  public static final boolean extensionInverted = false;

  public static final int extensionReduction = 36; // TODO figure out what this is?!?!?
  public static final int extensionCurrentLimit = 60;
  public static final double extensionKP = 1.08;
  public static final double extensionKI = 0.0;
  public static final double extensionKD = 1.50;
}

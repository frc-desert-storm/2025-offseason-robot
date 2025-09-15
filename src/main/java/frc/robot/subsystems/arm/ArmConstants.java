package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {

  // Pivot
  public static final int pivotLeftCanId = 21;
  public static final int pivotRightCanId = 22;

  public static final boolean pivotLeftInverted = false;
  public static final boolean pivotRightInverted = true;

  public static final int pivotReduction = 37;
  public static final int pivotCurrentLimit = 60;

  public static final double pivotMaxAccel = 200.0;
  public static final double pivotMaxVelo = 800.0;

  public static final double pivotRealKp = 5.0;
  public static final double pivotRealKd = 0.0;

  public static final double pivotRealKv = 0.0;
  public static final double pivotRealKg = 0.9;
  public static final double pivotRealKs = 0.0;
  public static final double pivotRealKa = 0.0;

  public static final double pivotSimKp = 1.0;
  public static final double pivotSimKd = 0.0;

  public static final double pivotSimKv = 0.0;
  public static final double pivotSimKg = 1.0;
  public static final double pivotSimKs = 0.0;
  public static final double pivotSimKa = 0.0;

  public static final DCMotor pivotGearbox = DCMotor.getNEO(1).withReduction(pivotReduction);

  // Extension
  public static final int extensionCanId = 23;

  public static final boolean extensionInverted = false;

  public static final double extensionReduction = 2.8125;
  public static final int extensionCurrentLimit = 60;

  public static final double extensionMaxAccel = 120.0;
  public static final double extensionMaxVelo = 600.0;

  public static final double extensionRealKp = 4.0;
  public static final double extensionRealKd = 0.0;

  public static final double extensionRealKv = 0.0;
  public static final double extensionRealKs = 0.7;
  public static final double extensionRealKa = 0.0;

  public static final double extensionSimKp = 30.0;
  public static final double extensionSimKd = 0.0;

  public static final double extensionSimKv = 0.0;
  public static final double extensionSimKs = 0.0;
  public static final double extensionSimKa = 0.0;

  public static final double maxExtensionInches = 23.3;  // Estimate from CAD, UPDATE
  public static final double minExtensionInches = 0.2;  // Guestimation of reasonable point to stop retracting arm

  public static final DCMotor extensionGearbox =
      DCMotor.getNEO(1).withReduction(extensionReduction);
  // Wrist
  public static final int wristCanId = 24;

  public static final boolean wristInverted = false;

  public static final double wristReduction = 11.57;
  public static final int wristCurrentLimit = 60;

  public static final double wristMaxAccel = 200.0;
  public static final double wristMaxVelo = 400.0;

  public static final double wristRealKp = 3.0;
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

  public static final int coralCanId = 31;
  public static final boolean coralInverted = false;
  public static final int coralCurrentLimit = 60;

  public static final DCMotor wristGearbox = DCMotor.getNEO(1).withReduction(wristReduction);
}

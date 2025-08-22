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
  public static final DCMotor pivotGearbox = DCMotor.getNEO(2).withReduction(pivotReduction);

  // Extension
  public static final int extensionCanId = 23;

  public static final boolean extensionInverted = false;

  public static final double extensionReduction = Units.inchesToMeters(2.8125);
  public static final int extensionCurrentLimit = 60;
  public static final DCMotor extensionGearbox =
      DCMotor.getNEO(1).withReduction(extensionReduction);
}

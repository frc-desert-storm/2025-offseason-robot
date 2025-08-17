package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ArmIOSparkMax implements ArmIO {

  private final SparkMax basePivotLeft = new SparkMax(basePivotLeftCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax basePivotRight = new SparkMax(basePivotRightCanID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax armExtender = new SparkMax(armExtenderCanID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax manipulator = new SparkMax(manipulatorCanId, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder basePivotLeftEncoder = basePivotLeft.getEncoder();
  private final RelativeEncoder basePivotRightEncoder = basePivotRight.getEncoder();
  private final RelativeEncoder armExtenderEncoder = armExtender.getEncoder();
  private final RelativeEncoder manipulatorEncoder = manipulator.getEncoder();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(realKs, realKv);

  private final PIDController frontLeftPIDController = new PIDController(realKp, 0, realKd);
  private final PIDController frontRightPIDController = new PIDController(realKp, 0, realKd);
  private final PIDController backLeftPIDController = new PIDController(realKp, 0, realKd);
  private final PIDController backRightPIDController = new PIDController(realKp, 0, realKd);

  public ArmIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(11.5);
    //     config
    //         .encoder
    //         .positionConversionFactor(2 * Math.PI / motorReduction) // Rotor Rotations -> Wheel
    // Radians
    //         .velocityConversionFactor(
    //             (2 * Math.PI) / 60.0 / motorReduction) // Rotor RPM -> Wheel Rad/Sec
    //         .uvwMeasurementPeriod(10)
    //         .uvwAverageDepth(2);

    config.inverted(basePivotLeftInverted);
    basePivotLeft.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(basePivotRightInverted);
    basePivotRight.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(armExtenderInverted);
    armExtender.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(manipulatorInverted);
    manipulator.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
}

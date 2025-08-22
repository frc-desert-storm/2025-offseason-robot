package frc.robot.subsystems.arm.pivot;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax pivotLeftMotor =
      new SparkMax(pivotLeftCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax pivotRightMotor =
      new SparkMax(pivotRightCanId, SparkLowLevel.MotorType.kBrushless);

  SparkClosedLoopController pivotLeftController = pivotLeftMotor.getClosedLoopController();
  SparkClosedLoopController pivotRightController = pivotRightMotor.getClosedLoopController();

  private Rotation2d targetAngle = new Rotation2d();

  private ArmFeedforward ff = new ArmFeedforward(0.0, 1.38, 0.0, 0.0);

  public PivotIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(pivotCurrentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(2 * Math.PI / pivotReduction) // Rotor Rotations -> Wheel Radians
        .velocityConversionFactor(
            (2 * Math.PI) / 60.0 / pivotReduction) // Rotor RPM -> Wheel Rad/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .p(0.03)
        .d(0.05)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(400)
        .maxAcceleration(1000)
        .allowedClosedLoopError(0.25);

    config.inverted(pivotLeftInverted);
    pivotLeftMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(pivotRightInverted);
    pivotRightMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    this.targetAngle = target;

    pivotLeftController.setReference(
        target.getRadians(),
        SparkBase.ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        ff.calculate(pivotLeftMotor.getEncoder().getPosition(), 0.0));
    pivotRightController.setReference(
        target.getRadians(),
        SparkBase.ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        ff.calculate(pivotRightMotor.getEncoder().getPosition(), 0.0));
  }

  @Override
  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // left motor
    inputs.pivotLeftPositionRad = pivotLeftMotor.getEncoder().getPosition();
    inputs.pivotLeftVelocityRadPerSec = pivotLeftMotor.getEncoder().getVelocity();
    inputs.pivotLeftAppliedVolts =
        pivotLeftMotor.getAppliedOutput() * pivotLeftMotor.getBusVoltage();
    inputs.pivotLeftCurrentAmps = pivotLeftMotor.getOutputCurrent();

    // right motor
    inputs.pivotRightPositionRad = pivotRightMotor.getEncoder().getPosition();
    inputs.pivotRightVelocityRadPerSec = pivotRightMotor.getEncoder().getVelocity();
    inputs.pivotRightAppliedVolts =
        pivotRightMotor.getAppliedOutput() * pivotRightMotor.getBusVoltage();
    inputs.pivotRightCurrentAmps = pivotRightMotor.getOutputCurrent();

    inputs.pivotAngle = targetAngle;
  }

  @Override
  public void resetPosition(Rotation2d pose) {
    pivotLeftMotor.getEncoder().setPosition(pose.getRadians());
    pivotRightMotor.getEncoder().setPosition(pose.getRadians());
  }
}

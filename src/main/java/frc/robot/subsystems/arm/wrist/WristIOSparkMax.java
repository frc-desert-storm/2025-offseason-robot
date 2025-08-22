package frc.robot.subsystems.arm.wrist;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class WristIOSparkMax implements WristIO {

  private final SparkMax wristMotor =
      new SparkMax(wristCanId, SparkLowLevel.MotorType.kBrushless);

  SparkClosedLoopController wristController = wristMotor.getClosedLoopController();

  private Rotation2d targetAngle = new Rotation2d();

  private final ArmFeedforward ff = new ArmFeedforward(0.0, 1.38, 0.0, 0.0);

  public WristIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(wristCurrentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(2 * Math.PI / wristReduction) // Rotor Rotations -> Wheel Radians
        .velocityConversionFactor(
            (2 * Math.PI) / 60.0 / wristReduction) // Rotor RPM -> Wheel Rad/Sec
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

    config.inverted(wristInverted);
    wristMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    this.targetAngle = target;

    wristController.setReference(
        target.getRadians(),
        SparkBase.ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        ff.calculate(wristMotor.getEncoder().getPosition(), 0.0));
  }

  @Override
  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void updateInputs(wristIOInputs inputs) {
    inputs.wristPositionRad = wristMotor.getEncoder().getPosition();
    inputs.wristVelocityRadPerSec = wristMotor.getEncoder().getVelocity();
    inputs.wristAppliedVolts =
        wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristCurrentAmps = wristMotor.getOutputCurrent();

    inputs.wristAngle = targetAngle;
  }

  @Override
  public void resetPosition(Rotation2d pose) {
    wristMotor.getEncoder().setPosition(pose.getRadians());
  }
}

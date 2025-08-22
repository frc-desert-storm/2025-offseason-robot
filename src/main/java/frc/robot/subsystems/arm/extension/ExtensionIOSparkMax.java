package frc.robot.subsystems.arm.extension;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ExtensionIOSparkMax implements ExtensionIO {

  private final SparkMax extensionMotor =
      new SparkMax(extensionCanId, SparkLowLevel.MotorType.kBrushless);

  SparkClosedLoopController extensionController = extensionMotor.getClosedLoopController();
  private double positionSetPoint = 0;

  public ExtensionIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(extensionCurrentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(extensionReduction) // Motor Rotations -> Extended Meters
        .velocityConversionFactor(60.0 / extensionReduction) // Rotor RPM -> Wheel Meters/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .p(0.001)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(100)
        .maxAcceleration(1000)
        .allowedClosedLoopError(Units.inchesToMeters(0.5));

    config.inverted(extensionInverted);
    extensionMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setTargetPosition(double position) {
    positionSetPoint = position;
    extensionController.setReference(
        positionSetPoint, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  @Override
  public double getTargetPosition() {
    return positionSetPoint;
  }

  @Override
  public void resetPose(double target) {
    extensionMotor.getEncoder().setPosition(target);
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    inputs.extensionPositionRad = extensionMotor.getEncoder().getPosition();
    inputs.extensionVelocityRadPerSec = extensionMotor.getEncoder().getVelocity();
    inputs.extensionAppliedVolts =
        extensionMotor.getAppliedOutput() * extensionMotor.getBusVoltage();
    inputs.extensionCurrentAmps = extensionMotor.getOutputCurrent();

    inputs.extensionSetpointMeters = positionSetPoint;
  }
}

package frc.robot.subsystems.arm.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.arm.ArmConstants;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax pivotLeftMotor =
      new SparkMax(ArmConstants.pivotLeftCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax pivotRightMotor =
      new SparkMax(ArmConstants.pivotRightCanId, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder encoder = pivotLeftMotor.getEncoder();

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(pivotKMaxVelocity, pivotKMaxAcceleration);

  private final ProfiledPIDController PID =
      new ProfiledPIDController(
          ArmConstants.pivotKP, ArmConstants.pivotKI, ArmConstants.pivotKD, constraints);

  private final ArmFeedforward ff =
      new ArmFeedforward(
          ArmConstants.pivotKS, ArmConstants.pivotKG,
          ArmConstants.pivotKV, ArmConstants.pivotKA);

  private Rotation2d targetAngle = new Rotation2d();

  public PivotIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(ArmConstants.pivotCurrentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(
            2 * Math.PI / ArmConstants.pivotReduction) // Rotor Rotations -> Wheel Radians
        .velocityConversionFactor(
            (2 * Math.PI) / 60.0 / ArmConstants.pivotReduction) // Rotor RPM -> Wheel Rad/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    config.inverted(ArmConstants.pivotLeftInverted);
    pivotLeftMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(ArmConstants.pivotRightInverted);
    config.follow(pivotLeftMotor);
    pivotRightMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    encoder.setPosition(0.0);
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    this.targetAngle = target;

    run();
  }

  @Override
  public void run() {
    final double ffOutput = ff.calculate(targetAngle.getRadians(), 0.0);
    final double pidOutput = PID.calculate(encoder.getPosition(), targetAngle.getRadians());
    pivotLeftMotor.setVoltage((ffOutput + pidOutput) * pivotLeftMotor.getBusVoltage());
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

    inputs.targetAngle = targetAngle;
  }
}

package frc.robot.subsystems.arm.pivot;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class PivotIOSparkMax implements PivotIO {

  private final SparkMax pivotLeftMotor =
      new SparkMax(pivotLeftCanId, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax pivotRightMotor =
      new SparkMax(pivotRightCanId, SparkLowLevel.MotorType.kBrushless);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(400, 200);

  private final ProfiledPIDController pid = new ProfiledPIDController(1.0, 0.0, 0.00, constraints);

  private final ArmFeedforward ff = new ArmFeedforward(0.0, 0.8, 0.0, 0.0);

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

    config.inverted(pivotLeftInverted);
    pivotLeftMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    config.inverted(pivotRightInverted);
    pivotRightMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    pid.setTolerance(0.25);
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    pid.setGoal(target.getRadians());
    run();
  }

  @Override
  public void run() {
    double pidOutput = pid.calculate(pivotLeftMotor.getEncoder().getPosition());

    double ffOutput = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);

    Logger.recordOutput("arm/pid", pidOutput);
    Logger.recordOutput("arm/ff", ffOutput);
    Logger.recordOutput("arm/setpoint", Units.radiansToDegrees(pid.getGoal().position));

    pivotLeftMotor.setVoltage(pidOutput + ffOutput);
    pivotRightMotor.setVoltage(pidOutput + ffOutput);
  }

  @Override
  public Rotation2d getTargetAngle() {
    return Rotation2d.fromRadians(pid.getSetpoint().position);
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

    inputs.pivotTargetAngle = getTargetAngle();

    run();
  }

  @Override
  public void resetPosition(Rotation2d pose) {
    pivotLeftMotor.getEncoder().setPosition(pose.getRadians());
    pivotRightMotor.getEncoder().setPosition(pose.getRadians());

    pid.setGoal(pose.getRadians());
  }
}

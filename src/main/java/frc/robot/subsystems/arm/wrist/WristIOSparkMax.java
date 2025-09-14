package frc.robot.subsystems.arm.wrist;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.subsystems.arm.ArmConstants.wristRealKa;
import static frc.robot.subsystems.arm.ArmConstants.wristRealKd;
import static frc.robot.subsystems.arm.ArmConstants.wristRealKg;
import static frc.robot.subsystems.arm.ArmConstants.wristRealKp;
import static frc.robot.subsystems.arm.ArmConstants.wristRealKv;

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

public class WristIOSparkMax implements WristIO {

  private final SparkMax wristMotor = new SparkMax(wristCanId, SparkLowLevel.MotorType.kBrushless);

  TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(wristMaxVelo, wristMaxAccel);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(wristRealKp, 0.0, wristRealKd, constraints);

  private final ArmFeedforward ff =
      new ArmFeedforward(wristRealKs, wristRealKg, wristRealKv, wristRealKa);

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

    config.inverted(wristInverted);
    wristMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    wristMotor.getEncoder().setPosition(0);
    pid.setTolerance(0.2);
  }

  @Override
  public void setTargetAngle(Rotation2d target) {
    pid.setGoal(target.getRadians());
    run();
  }

  @Override
  public void run() {
    double pidOutput = pid.calculate(wristMotor.getEncoder().getPosition());

    double ffOutput = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);

    Logger.recordOutput("arm/wrist/pid", pidOutput);
    Logger.recordOutput("arm/wrist/ff", ffOutput);
    Logger.recordOutput("arm/wrist/setpoint", Units.radiansToDegrees(pid.getGoal().position));

    wristMotor.setVoltage(pidOutput + ffOutput);
  }

  @Override
  public Rotation2d getTargetAngle() {
    return Rotation2d.fromRadians(pid.getGoal().position);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPositionRad = wristMotor.getEncoder().getPosition();
    inputs.wristVelocityRadPerSec = wristMotor.getEncoder().getVelocity();
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristCurrentAmps = wristMotor.getOutputCurrent();

    inputs.wristAngle = Rotation2d.fromRadians(pid.getGoal().position);

    run();
  }

  @Override
  public void resetPosition(Rotation2d pose) {
    wristMotor.getEncoder().setPosition(pose.getRadians());

    pid.setGoal(0);
  }

  @Override
  public boolean atGoal() {
    return pid.atGoal();
  }
}

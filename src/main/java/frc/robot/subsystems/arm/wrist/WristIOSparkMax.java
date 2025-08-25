package frc.robot.subsystems.arm.wrist;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.spark.*;
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

  private Rotation2d targetAngle = new Rotation2d();

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(400, 200);

  private final ProfiledPIDController pid = new ProfiledPIDController(1.0, 0.0, 0.00, constraints);

  private final ArmFeedforward ff = new ArmFeedforward(0.0, 0.8, 0.0, 0.0);

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

    Logger.recordOutput("wrist/pid", pidOutput);
    Logger.recordOutput("wrist/ff", ffOutput);
    Logger.recordOutput("wrist/setpoint", Units.radiansToDegrees(pid.getGoal().position));

    wristMotor.setVoltage(pidOutput + ffOutput);
  }

  @Override
  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPositionRad = wristMotor.getEncoder().getPosition();
    inputs.wristVelocityRadPerSec = wristMotor.getEncoder().getVelocity();
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristCurrentAmps = wristMotor.getOutputCurrent();

    inputs.wristAngle = targetAngle;
  }

  @Override
  public void resetPosition(Rotation2d pose) {
    wristMotor.getEncoder().setPosition(pose.getRadians());
  }
}

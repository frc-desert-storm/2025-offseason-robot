package frc.robot.subsystems.arm.extension;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.subsystems.arm.ArmConstants.extensionRealKa;
import static frc.robot.subsystems.arm.ArmConstants.extensionRealKd;
import static frc.robot.subsystems.arm.ArmConstants.extensionRealKp;
import static frc.robot.subsystems.arm.ArmConstants.extensionRealKv;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class ExtensionIOSparkMax implements ExtensionIO {

  private final SparkMax extensionMotor =
      new SparkMax(extensionCanId, SparkLowLevel.MotorType.kBrushless);

  TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(extensionMaxVelo, extensionMaxAccel);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(extensionRealKp, 0.0, extensionRealKd, constraints);

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(extensionRealKs, extensionRealKv, extensionRealKa);

  public ExtensionIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(extensionCurrentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(extensionReduction) // Rotor Rotations -> Extension meters
        .velocityConversionFactor(60.0 / extensionReduction) // Rotor RPM -> Extension meters/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    config.inverted(extensionInverted);
    extensionMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setTargetPosition(double positionInMeters) {
    pid.setGoal(positionInMeters);
    run();
  }

  @Override
  public void run() {
    double pidOutput = pid.calculate(extensionMotor.getEncoder().getPosition());

    double ffOutput =
        ff.calculateWithVelocities(pid.getSetpoint().position, pid.getSetpoint().velocity);

    Logger.recordOutput("arm/extension/pid", pidOutput);
    Logger.recordOutput("arm/extension/ff", ffOutput);
    Logger.recordOutput("arm/extension/setpoint", pid.getGoal().position);

    extensionMotor.setVoltage(pidOutput + ffOutput);
  }

  @Override
  public double getTargetPosition() {
    return pid.getSetpoint().position;
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    inputs.extensionPositionMeters = extensionMotor.getEncoder().getPosition();
    inputs.extensionVelocityMetersPerSec = extensionMotor.getEncoder().getVelocity();
    inputs.extensionAppliedVolts =
        extensionMotor.getAppliedOutput() * extensionMotor.getBusVoltage();
    inputs.extensionCurrentAmps = extensionMotor.getOutputCurrent();

    inputs.extensionSetpointMeters = pid.getSetpoint().position;

    run();
  }

  @Override
  public void resetPosition(double positionInMeters) {
    extensionMotor.getEncoder().setPosition(positionInMeters);

    pid.setGoal(positionInMeters);
  }
  
  @Override
  public boolean atGoal(){
    return pid.atGoal();
  }
}

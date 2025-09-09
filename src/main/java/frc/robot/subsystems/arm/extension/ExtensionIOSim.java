package frc.robot.subsystems.arm.extension;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class ExtensionIOSim implements ExtensionIO {

  private final SparkMax extensionMotor =
      new SparkMax(extensionCanId, SparkLowLevel.MotorType.kBrushless);

  private final SparkMaxSim extensionMotorSim = new SparkMaxSim(extensionMotor, extensionGearbox);

  TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(extensionMaxVelo, extensionMaxAccel);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(extensionSimKp, 0.0, extensionSimKd, constraints);

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(extensionSimKs, extensionSimKv, extensionSimKa);

  public ExtensionIOSim() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(extensionCurrentLimit)
        .voltageCompensation(11.5);
    config
        .encoder
        .positionConversionFactor(extensionReduction) // Rotor Rotations -> Extension inches
        .velocityConversionFactor(60.0 / extensionReduction) // Rotor RPM -> Extension inches/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    config.inverted(extensionInverted);
    extensionMotor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void setTargetPosition(double positionInInches) {
    pid.setGoal(positionInInches);
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

    extensionMotorSim.iterate(pidOutput + ffOutput, extensionMotorSim.getBusVoltage(), 0.2);
  }

  @Override
  public double getTargetPosition() {
    return pid.getGoal().position;
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    inputs.extensionPositionInches = extensionMotor.getEncoder().getPosition();
    inputs.extensionVelocityInchesPerSec = extensionMotor.getEncoder().getVelocity();
    inputs.extensionAppliedVolts =
        extensionMotorSim.getAppliedOutput() * extensionMotorSim.getBusVoltage();
    inputs.extensionCurrentAmps = extensionMotorSim.getMotorCurrent();

    inputs.extensionSetpointInches = pid.getSetpoint().position;

    run();
  }

  @Override
  public void resetPosition(double positionInInches) {
    extensionMotor.getEncoder().setPosition(positionInInches);

    pid.setGoal(positionInInches);
  }

  @Override
  public boolean atGoal() {
    return pid.atGoal();
  }
}

package frc.robot.subsystems.coral;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralIOSparkMax implements CoralIO {
  private final SparkMax motor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(0);

  public CoralIOSparkMax() {
    var config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(wristCurrentLimit)
        .voltageCompensation(11.5);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    config.inverted(wristInverted);
    motor.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.limitSwitch = limitSwitch.get();

    inputs.coralAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.coralCurrentAmps = motor.getOutputCurrent();
    inputs.coralVelocityRadPerSec = motor.getEncoder().getVelocity();
    inputs.coralPositionRad = motor.getEncoder().getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}

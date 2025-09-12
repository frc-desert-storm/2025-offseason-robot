package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
  @AutoLog
  public static class CoralIOInputs {
    public double coralPositionRad = 0.0;
    public double coralVelocityRadPerSec = 0.0;
    public double coralAppliedVolts = 0.0;
    public double coralCurrentAmps = 0.0;
    public boolean limitSwitch = false;
  }

  public default void updateInputs(CoralIOInputs inputs) {}

  /** Set the voltage of the coral manipulator */
  public default void setVoltage(double volts) {}
}

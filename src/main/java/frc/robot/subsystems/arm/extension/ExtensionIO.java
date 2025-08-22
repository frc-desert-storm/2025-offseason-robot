package frc.robot.subsystems.arm.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  default void updateInputs(ExtensionIOInputs inputs) {}

  @AutoLog
  class ExtensionIOInputs {
    public double extensionPositionInMeters;

    public double extensionPositionRad = 0.0;
    public double extensionVelocityRadPerSec = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionCurrentAmps = 0.0;
  }

  default void setTargetPosition(double positionInMeters) {}

  default double getTargetPosition(){
    return 0;
  }
}

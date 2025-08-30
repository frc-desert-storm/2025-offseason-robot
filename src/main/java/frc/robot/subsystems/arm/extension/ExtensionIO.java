package frc.robot.subsystems.arm.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  default void updateInputs(ExtensionIOInputs inputs) {}

  @AutoLog
  class ExtensionIOInputs {
    public double extensionSetpointMeters = 0.0;

    public double extensionPositionMeters = 0.0;
    public double extensionVelocityMetersPerSec = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionCurrentAmps = 0.0;
  }

  default void setTargetPosition(double positionInMeters) {}

  default void run() {}

  default double getTargetPosition() {
    return 0;
  }

  default void resetPosition(double positionInMeters) {}
}

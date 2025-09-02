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

  /**
   * Sets the extension setpoint.
   *
   * @param positionInMeters The position in meters to set.
   */
  default void setTargetPosition(double positionInMeters) {}

  /**
   * Gets the current target position.
   *
   * @return The target position in meters.
   */
  default double getTargetPosition() {
    return 0;
  }

  /**
   * Resets the extension to the target position.
   *
   * @param positionInMeters The target position in meters.
   */
  default void resetPosition(double positionInMeters) {}

  /** Returns if it's at the pid goal. */
  default boolean atGoal() {
    return false;
  }

  default void run() {}
}

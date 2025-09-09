package frc.robot.subsystems.arm.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  default void updateInputs(ExtensionIOInputs inputs) {}

  @AutoLog
  class ExtensionIOInputs {
    public double extensionSetpointInches = 0.0;

    public double extensionPositionInches = 0.0;
    public double extensionVelocityInchesPerSec = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionCurrentAmps = 0.0;
  }

  /**
   * Sets the extension setpoint.
   *
   * @param positionInInches The position in inches to set.
   */
  default void setTargetPosition(double positionInInches) {}

  /**
   * Gets the current target position.
   *
   * @return The target position in inches.
   */
  default double getTargetPosition() {
    return 0;
  }

  /**
   * Resets the extension to the target position.
   *
   * @param positionInInches The target position in inches.
   */
  default void resetPosition(double positionInInches) {}

  /** Returns if it's at the pid goal. */
  default boolean atGoal() {
    return false;
  }

  default void run() {}
}

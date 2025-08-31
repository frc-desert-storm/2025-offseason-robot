package frc.robot.subsystems.arm.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  default void updateInputs(WristIOInputs inputs) {}

  @AutoLog
  class WristIOInputs {
    public Rotation2d wristAngle = Rotation2d.kZero;

    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  /** Returns the angle of the wrist. */
  default Rotation2d getTargetAngle() {
    return null;
  }

  /**
   * Sets the angle of the wrist.
   *
   * @param target The target angle to set.
   */
  default void setTargetAngle(Rotation2d target) {}

  /**
   * Resets the wrist to the target angle.
   *
   * @param pose The target angle to reset to.
   */
  default void resetPosition(Rotation2d pose) {}

  default void run() {}
}

package frc.robot.subsystems.arm.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  default void updateInputs(wristIOInputs inputs) {}

  @AutoLog
  class wristIOInputs {
    public Rotation2d wristAngle = Rotation2d.kZero;

    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  default Rotation2d getTargetAngle() {
    return null;
  }

  default void setTargetAngle(Rotation2d target) {}

  default void resetPosition(Rotation2d pose) {}
}

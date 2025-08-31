package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  default void updateInputs(PivotIOInputs inputs) {}

  @AutoLog
  class PivotIOInputs {
    public Rotation2d pivotTargetAngle = Rotation2d.kZero;

      public Rotation2d pivotEncoderPosition = Rotation2d.kZero;

    public double pivotLeftPositionRad = 0.0;
    public double pivotLeftVelocityRadPerSec = 0.0;
    public double pivotLeftAppliedVolts = 0.0;
    public double pivotLeftCurrentAmps = 0.0;

    public double pivotRightPositionRad = 0.0;
    public double pivotRightVelocityRadPerSec = 0.0;
    public double pivotRightAppliedVolts = 0.0;
    public double pivotRightCurrentAmps = 0.0;
  }

  default Rotation2d getTargetAngle() {
    return null;
  }

  default void setTargetAngle(Rotation2d target) {}

  default void resetPosition(Rotation2d pose) {}

  default void run() {}
}

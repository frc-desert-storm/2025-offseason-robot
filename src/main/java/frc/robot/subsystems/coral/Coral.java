package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  public Coral(CoralIO io) {
    this.io = io;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getCurrent() {
    return inputs.coralCurrentAmps;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);
  }
}

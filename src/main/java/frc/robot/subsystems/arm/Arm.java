package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  public Arm() {}
    // The P gain for the PID controller that drives this arm.
  private double m_armKp = ArmConstants.kDefaultArmKp;
  private double m_armSetpointDegrees = ArmConstants.kDefaultArmSetpointDegrees;
  private double m_extensionSetpoint = ArmConstants.kArmLength;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  // private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);
  private final DCMotor m_extenderMotor = DCMotor.getNEO(1);
  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
  private final Encoder m_encoder =
      new Encoder(ArmConstants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final Encoder m_extenderEncoder = new Encoder(2,3);
  private final SparkMax m_motor = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
  private final PWMSparkMax m_extender = new PWMSparkMax(1);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
          Constants.kArmLength,
          Constants.kMinAngleRads,
          Constants.kMaxAngleRads,
          true,
          0,
          Constants.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final EncoderSim m_extenderEncoderSim = new EncoderSim(m_extenderEncoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(120, 120);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 10, 5);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 5, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              Units.metersToInches(Constants.kArmLength),
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Subsystem constructor. */
  public Arm() {
    m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    m_extenderEncoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
    Preferences.initDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    Preferences.initDouble(Constants.kArmPKey, m_armKp);
  }

  public void setSetpointDegrees(double setPoint){
    m_armSetpointDegrees = setPoint;
  }

  public double getSetpointDegress() {
    return m_armSetpointDegrees;
  }

  // public void setExtensionSetpoint(double num){
  //   if(!(m_extensionSetpoint == 8 && num < 0)){
  //     m_extensionSetpoint += num;
  //   }
  // }

  public void moveArmUp(){
    if(m_armSetpointDegrees < 90)
    m_armSetpointDegrees += 2;
  }

  public void moveArmDown(){
    if(m_armSetpointDegrees > 0)
    m_armSetpointDegrees -= 2;
  }

  public void extendArm(){
    m_extensionSetpoint += Units.inchesToMeters(2);
  }

  public void retractArm(){
    m_extensionSetpoint -= Units.inchesToMeters(2);
  }

  public void resetPose(){
    // m_arm.setAngle(0);
    // reachSetpoint();
    m_encoder.reset();
    m_encoderSim.resetData();
    m_armSetpointDegrees = 2;
    m_armSim.setState(0, 0);
    m_arm.setAngle(m_armSetpointDegrees);
  }
  
  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    // m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    // m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
        
    // Update the Mechanism Arm angle based on the simulated arm angle
    // m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    // m_arm.setLength(m_extensionSetpoint);
  }

  public void robotPeriodic(){
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    m_extenderEncoderSim.setDistance(m_arm.getLength());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    
    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    m_arm.setLength(m_extensionSetpoint);
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpointDegrees = Preferences.getDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
      m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
      m_controller.setP(m_armKp);
    }
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void reachSetpoint() {
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), Units.degreesToRadians(m_armSetpointDegrees));
    m_motor.setVoltage(pidOutput);
  }

  public void reachExtensionpoint(){
    var pidOutput = m_controller.calculate(m_extenderEncoder.getDistance(), m_extensionSetpoint);
    m_extender.setVoltage(pidOutput);
  }

  public void stop() {
    m_motor.set(0.0);
  }

  @Override
  public void close() {
    m_motor.close();
    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_controller.close();
    m_arm.close();
  }
}

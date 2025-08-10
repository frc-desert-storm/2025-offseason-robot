// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final MecanumDrivetrain m_mecanum = new MecanumDrivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_mecanum.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    double xSpeed = 0.0;

    if(-m_controller.getLeftY() > 0.1 || -m_controller.getLeftY() < -0.1){
        xSpeed = -m_xspeedLimiter.calculate(m_controller.getLeftY()) * MecanumDrivetrain.kMaxLinearSpeed;
    }

    double ySpeed = 0.0;

    if(-m_controller.getLeftX() > 0.1 || -m_controller.getLeftX() < -0.1){
        ySpeed = -m_yspeedLimiter.calculate(m_controller.getLeftX()) * MecanumDrivetrain.kMaxLinearSpeed;
    }
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = 0.0;

    if(-m_controller.getRightX() > 0.1 || -m_controller.getRightX() < -0.1){
        rot = -m_rotLimiter.calculate(m_controller.getRightX()) * MecanumDrivetrain.kMaxAngularSpeed;
    }

    SmartDashboard.putNumber("Forward", xSpeed);
    SmartDashboard.putNumber("Left/Right", ySpeed);
    SmartDashboard.putNumber("Rotation", rot);
    m_mecanum.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}

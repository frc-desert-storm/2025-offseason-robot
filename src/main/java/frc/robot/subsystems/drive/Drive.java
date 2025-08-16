package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final MecanumDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field = new Field2d();

  public Drive(DriveIO io, GyroIO gyroIO) {
    this.io = io;
    this.gyroIO = gyroIO;

    m_poseEstimator =
        new MecanumDrivePoseEstimator(
            m_kinematics,
            gyroInputs.yawPosition,
            io.getCurrentDistances(),
            Pose2d.kZero,
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Drive/Gyro", inputs);

    m_poseEstimator.update(gyroInputs.yawPosition, io.getCurrentDistances());
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                0.02));
    mecanumDriveWheelSpeeds.desaturate(kMaxLinearSpeed);
    io.setSpeeds(mecanumDriveWheelSpeeds);
  }

  public void resetPose(Pose2d pose){
    m_poseEstimator.resetPose(pose);
    m_field.setRobotPose(pose);
    gyroIO.resetRotation(pose.getRotation());
  }
}

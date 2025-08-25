package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

    AutoBuilder.configure(
        m_poseEstimator::getEstimatedPosition, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        ppConfig, // The robot configuration
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
        },
        this // Reference to this subsystem to set requirements
        );

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Drive/Gyro", inputs);

    m_poseEstimator.update(gyroInputs.yawPosition, io.getCurrentDistances());

    Logger.recordOutput("Odometry/Pose", m_poseEstimator.getEstimatedPosition());
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

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
    gyroIO.resetRotation(pose.getRotation());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var mecanumDriveWheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    mecanumDriveWheelSpeeds.desaturate(kMaxLinearSpeed);
    io.setSpeeds(mecanumDriveWheelSpeeds);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new MecanumDriveWheelSpeeds(
            inputs.frontLeftVelocityRadPerSec * wheelRadiusMeters,
            inputs.frontRightVelocityRadPerSec * wheelRadiusMeters,
            inputs.backLeftVelocityRadPerSec * wheelRadiusMeters,
            inputs.backRightVelocityRadPerSec * wheelRadiusMeters));
  }
}

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModule;
import frc.robot.constants.SwerveConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveDriveOdometry m_swerveOdometry;
  public SwerveModule[] m_swerveMods;
  public SwerveModuleState[] m_desiredStates;

  public AHRS m_gyro;
  public StructArrayPublisher<SwerveModuleState> publisher;
  public StructArrayPublisher<SwerveModuleState> publisher2;

  public SwerveSubsystem() {
    m_gyro = new AHRS(NavXComType.kMXP_SPI);
    initialHeadingReset();

    m_swerveMods = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.Mod0.CONSTANTS),
      new SwerveModule(1, SwerveConstants.Mod1.CONSTANTS),
      new SwerveModule(2, SwerveConstants.Mod2.CONSTANTS),
      new SwerveModule(3, SwerveConstants.Mod3.CONSTANTS)
    };

    m_swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getGyroYaw(), getModulePositions());
    publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    publisher2 = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct)
      .publish();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
        // ROBOT RELATIVE ChassisSpeeds. Also
        // optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController(
          // PPHolonomicController is the built in path following controller
          // for holonomic drive trains
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
    m_desiredStates = swerveModuleStates;

    for (SwerveModule mod : m_swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  private void initialHeadingReset() {
    try {
      Thread.sleep(1000);
    } catch (Exception e) {}

    m_gyro.reset();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
    // publisher2.set(+desiredStates);

    for (SwerveModule mod : m_swerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_swerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    double weight = 0.24010657343652914 * 4;/** Erorr of the chassis distance */

    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_swerveMods) {
      positions[mod.moduleNumber] = new SwerveModulePosition(
        mod.getPosition().distanceMeters * weight,
        mod.getPosition().angle
      );
    }
    return positions;
  }

  public Pose2d getPose() {
    return m_swerveOdometry.getPoseMeters();
  }

  public void resetPose() {
    m_swerveOdometry.resetPose(Pose2d.kZero);
  }

  public void setPose(Pose2d pose) {
    m_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public void lockWheels() {
    m_swerveMods[0].forceSetAngle(Rotation2d.fromDegrees(45));
    m_swerveMods[1].forceSetAngle(Rotation2d.fromDegrees(-45));
    m_swerveMods[2].forceSetAngle(Rotation2d.fromDegrees(-45));
    m_swerveMods[3].forceSetAngle(Rotation2d.fromDegrees(45));
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    m_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    m_swerveOdometry.resetPosition(
      getGyroYaw(),
      getModulePositions(),
      new Pose2d(getPose().getTranslation(), new Rotation2d())
    );
  }

  public Rotation2d getGyroYaw() {
    return (
      (SwerveConstants.INVERT_GYRO)
        ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
        : Rotation2d.fromDegrees(m_gyro.getYaw())
    );
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : m_swerveMods) {
      mod.resetToAbsolute();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return SwerveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule mod : m_swerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }
  }

  public double getFrontLeftMotorDistance() {
    return Conversions.rotationsToMeters(
      m_swerveMods[0].getDriveMotor().getPosition().getValueAsDouble(),
      SwerveConstants.WHEEL_CIRCUMFERENCE
    );
  }

  public String getDriveMotorDistances() {
    StringBuilder builder = new StringBuilder();
    for (var mod : m_swerveMods) {
      builder.append(
        Conversions.rotationsToMeters(
          mod.getDriveMotor().getPosition().getValueAsDouble(),
          SwerveConstants.WHEEL_CIRCUMFERENCE
        )
      );
      builder.append(", ");
    }

    return builder.toString();
  }

  public void resetDriveEncoders() {
    for (var module : m_swerveMods) {
      module.resetDriveEncoder();
    }
  }

  public void moveFrontLeft() {
    m_swerveMods[0].getDriveMotor().set(0.1);
  }

  @Override
  public void periodic() {
    m_swerveOdometry.update(getGyroYaw(), getModulePositions());
    for (SwerveModule mod : m_swerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    SmartDashboard.putNumber("YAW", m_gyro.getYaw());
    SmartDashboard.putNumber("PITCH", m_gyro.getPitch());
    SmartDashboard.putNumber("ROLL", m_gyro.getRoll());
    SmartDashboard.putNumber("ANGLE", m_gyro.getAngle());
    SmartDashboard.putString("CURRENT_HEADING", getHeading().toString());

    SmartDashboard.putString("POSITION", getPose().toString());
    SmartDashboard.putString("Module positions", getDriveMotorDistances());
    SmartDashboard.putNumber("Front Left Mod Distance", getFrontLeftMotorDistance());
    if (m_desiredStates != null) {
      publisher2.set(m_desiredStates);
    }
    publisher.set(getModuleStates());
    Logger.recordOutput("Odometry/2025_Robot", getPose());
  } 
}

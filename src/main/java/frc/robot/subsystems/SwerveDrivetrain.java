package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import frc.robot.subsystems.swerve.Chassis;
import frc.robot.subsystems.swerve.Odometry;
import frc.robot.subsystems.swerve.PathFollowing;
import frc.robot.utils.Navx;

public class SwerveDrivetrain extends SubsystemBase implements Chassis {
  private static final Pose2d DEVANT_TAG_0 = new Pose2d(1.73, 3.5, Rotation2d.fromDegrees(90));

  /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    private static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6858;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    private static final double DRIVETRAIN_WHEELBASE_METERS = DRIVETRAIN_TRACKWIDTH_METERS;

    private static final int FRONT_LEFT = 0;
    private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 3;
    private static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 7;
    private static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 9;
    private static final double FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(240.28);

    private static final int FRONT_RIGHT = 1;
    private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
    private static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 12;
    private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(189.49);

    private static final int BACK_LEFT = 2;
    private static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 8;
    private static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 11;
    private static final double BACK_LEFT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(178.23);

    private static final int BACK_RIGHT = 3;
    private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 6;
    private static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 4;
    private static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 0;
    private static final double BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS = -Math.toRadians(36.55);

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  private static final double MAX_VOLTAGE = 12.0;

  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  private static final double MAX_VELOCITY_METERS_PER_SECOND = // Env 3.23 m/s
    5000.0 / 60.0 *
    SdsModuleConfigurations.MK4_L1.getDriveReduction() * // (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI; // 0.10033

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d( DRIVETRAIN_TRACKWIDTH_METERS / 2.0,  DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d( DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,  DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );
  private final SwerveModuleState[] m_stop_states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final AHRS m_navx = Navx.newReadyNavx();
  private final Field2d m_fieldTracker = new Field2d();

  private final ShuffleboardTab m_dashboardTab = Shuffleboard.getTab("Drivetrain");
  private final SwerveModule[] m_modules = {
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0),
      Mk4SwerveModuleHelper.GearRatio.L1,
      FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
      FRONT_LEFT_MODULE_STEER_MOTOR_ID,
      FRONT_LEFT_MODULE_STEER_ENCODER_ID,
      FRONT_LEFT_MODULE_STEER_OFFSET_RADIANS
    ),
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0),
      Mk4SwerveModuleHelper.GearRatio.L1,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
      FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
      FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
      FRONT_RIGHT_MODULE_STEER_OFFSET_RADIANS
    ),
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0),
      Mk4SwerveModuleHelper.GearRatio.L1,
      BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
      BACK_LEFT_MODULE_STEER_MOTOR_ID,
      BACK_LEFT_MODULE_STEER_ENCODER_ID,
      BACK_LEFT_MODULE_STEER_OFFSET_RADIANS
    ),
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0),
      Mk4SwerveModuleHelper.GearRatio.L1,
      BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
      BACK_RIGHT_MODULE_STEER_MOTOR_ID,
      BACK_RIGHT_MODULE_STEER_ENCODER_ID,
      BACK_RIGHT_MODULE_STEER_OFFSET_RADIANS
    )
  };
  private final Odometry m_odometry = new Odometry(getGyroscopeRotation(), getModulePositions(), m_kinematics, m_fieldTracker);
  private final PathFollowing m_pathFollowing = new PathFollowing(this, m_kinematics, m_fieldTracker);
  private final XboxController m_gamepad;

  private SwerveModuleState[] m_states = m_stop_states;
  // On stocke la dernière position pour la télémétrie, car c'est "cher" à aller chercher
  // (cela requiert une requête sur le bus CAN pour tous les encodeurs)
  private SwerveModulePosition[] m_lastPosition = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private boolean m_canUseFusedHeading = false;

  public SwerveDrivetrain(XboxController gamepad) {
    this.m_gamepad = gamepad;
    setDefaultCommand(drive());
    SmartDashboard.putData(this);
    SmartDashboard.putData(m_fieldTracker);
  }

  private double withDeadBand(double setPoint) {
    final var epsilon = 0.05;
    if (Math.abs(setPoint) < epsilon) {
      return 0.0;
    }
    return setPoint;
  }

  public Command drive() {
    return run(() -> {
        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          withDeadBand(-m_gamepad.getLeftX()),
          withDeadBand(-m_gamepad.getLeftY()),
          withDeadBand(m_gamepad.getRightX()),
          getGyroscopeRotation()
        );
        m_states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
      })
      .andThen(this::stopMotors)
      .withName("piloter");
  }

  public Command goToFrontOfTag0() {
    return m_pathFollowing.goTo(DEVANT_TAG_0);
  }

  public Command zigzag() {
    return m_pathFollowing.robotRelativeMove(
      List.of(
        new Translation2d(0.5, 0.5),
        new Translation2d(1, -0.5)
      ),
      new Pose2d(1.5, 0, Rotation2d.fromDegrees(0))
    );
  }

  public void stopMotors() {
    m_states = m_stop_states;
  }

  public Pose2d odometryEstimation() {
    return m_odometry.getPoseM();
  }

  private Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      m_canUseFusedHeading = true;
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    // Also go from [-180, 180] to [0, 360]
    return Rotation2d.fromDegrees(180 - m_navx.getYaw());
  }

  private SwerveModulePosition[] getModulePositions() {
    var result = new SwerveModulePosition[4];

    for(int i=0; i<m_modules.length; i++) {
      var module = m_modules[i];
      result[i] = new SwerveModulePosition(
        module.getPositionMeters(),
        Rotation2d.fromRadians(module.getSteerAngle())
      );
    }

    m_lastPosition = result;

    return result;
  }

  @Override
  public void setModuleStates(SwerveModuleState[] states) {
    m_states = states;
  }

  @Override
  public void periodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);

    for(int i=0; i<m_modules.length; i++) {
      var moduleState = m_states[i];
      m_modules[i].set(
        moduleState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        moduleState.angle.getRadians()
      );
    }

    m_odometry.update(getGyroscopeRotation(), getModulePositions());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("angleGyro", () -> getGyroscopeRotation().getDegrees(), null);
      builder.addBooleanProperty("fuseHeading", () -> m_canUseFusedHeading, null);
  }
}

package frc.robot.subsystems;

import java.util.List;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.ModuleConfigurationProvider;
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
    private static final double TRACKWIDTH_M = 0.6858;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    private static final double WHEELBASE_M = TRACKWIDTH_M;

    private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 3;
    private static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 7;
    private static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 9;
    private static final double FRONT_LEFT_MODULE_STEER_OFFSET_RAD = -Math.toRadians(240.28);

    private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
    private static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 12;
    private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_RAD = -Math.toRadians(189.49);

    private static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 8;
    private static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 11;
    private static final double BACK_LEFT_MODULE_STEER_OFFSET_RAD = -Math.toRadians(178.23);

    private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 6;
    private static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 4;
    private static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 0;
    private static final double BACK_RIGHT_MODULE_STEER_OFFSET_RAD = -Math.toRadians(36.55);

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * 
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  private static final double MAX_VOLTAGE = 12.0;

  // TODO: mesurer
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * 
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  private static final double MAX_VELOCITY_MS = // = 3.225 m/s
    5000.0 / 60.0 * // = 83.33 rot/s
    SdsModuleConfigurations.MK4_L1.getDriveReduction() * // (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0) = 0.1228
    SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI; // = 0.10033 * PI =  0.315196 m/rot

  //valeur intiale:0.10033 -- 0.3085 est la distance par tour mesuree.  
  private static final ModuleConfigurationProvider GEAR_RATIO = Mk4SwerveModuleHelper.GearRatio.L1.withWheelDiameter(0.3085 / Math.PI);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d( TRACKWIDTH_M / 2.0,  WHEELBASE_M / 2.0),
    // Front right
    new Translation2d( TRACKWIDTH_M / 2.0, -WHEELBASE_M / 2.0),
    // Back left
    new Translation2d(-TRACKWIDTH_M / 2.0,  WHEELBASE_M / 2.0),
    // Back right
    new Translation2d(-TRACKWIDTH_M / 2.0, -WHEELBASE_M / 2.0)
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
      GEAR_RATIO,
      FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
      FRONT_LEFT_MODULE_STEER_MOTOR_ID,
      FRONT_LEFT_MODULE_STEER_ENCODER_ID,
      FRONT_LEFT_MODULE_STEER_OFFSET_RAD
    ),
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0),
      GEAR_RATIO,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
      FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
      FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
      FRONT_RIGHT_MODULE_STEER_OFFSET_RAD
    ),
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0),
      GEAR_RATIO,
      BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
      BACK_LEFT_MODULE_STEER_MOTOR_ID,
      BACK_LEFT_MODULE_STEER_ENCODER_ID,
      BACK_LEFT_MODULE_STEER_OFFSET_RAD
    ),
    Mk4SwerveModuleHelper.createFalcon500(
      m_dashboardTab
        .getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0),
      GEAR_RATIO,
      BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
      BACK_RIGHT_MODULE_STEER_MOTOR_ID,
      BACK_RIGHT_MODULE_STEER_ENCODER_ID,
      BACK_RIGHT_MODULE_STEER_OFFSET_RAD
    )
  };
  private final Odometry m_odometry = new Odometry(getGyroscopeRotation(), getModulePositions(), m_kinematics, m_fieldTracker);
  private final PathFollowing m_pathFollowing = new PathFollowing(this, m_kinematics, m_fieldTracker);
  private final CommandXboxController m_gamepad;

  private SwerveModuleState[] m_states = m_stop_states;
  private ChassisSpeeds m_actualSpeeds = new ChassisSpeeds();
  private boolean m_canUseFusedHeading = false;

  public SwerveDrivetrain(CommandXboxController gamepad) {
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
          withDeadBand(m_gamepad.getLeftX()),
          withDeadBand(-m_gamepad.getLeftY()),
          withDeadBand(-m_gamepad.getRightX()),
          m_odometry.getPoseM().getRotation()
        );
        setModuleStates(m_kinematics.toSwerveModuleStates(chassisSpeeds));
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

  public Command forward1MeterCommand() {
    return m_pathFollowing.robotRelativeMove(
      List.of(),
      new Pose2d(1, 0, Rotation2d.fromDegrees(0))
    );
  }

  public Command activateCameraEstimation() {
    return runOnce(m_odometry::activateCameraEstimation);
  }

  public Command deactivateCameraEstimation() {
    return runOnce(m_odometry::deactivateCameraEstimation);
  }

  public Command incrementCameraLatencyCompensation() {
    return runOnce(m_odometry::incrementLatencyCompensation);
  }

  public Command decrementCameraLatencyCompensation() {
    return runOnce(m_odometry::decrementLatencyCompensation);
  }

  public void stopMotors() {
    setModuleStates(m_stop_states);
  }

  public Pose2d odometryEstimation() {
    return m_odometry.getPoseM();
  }

  private Rotation2d getGyroscopeRotation() {
      return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
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

    return result;
  }

  @Override
  public void setModuleStates(SwerveModuleState[] states) {
    m_states = states;
  }

  private void updateActualSpeeds() {
    var actualStates = new SwerveModuleState[4];
    for(int i=0; i<m_modules.length; i++) {
      var module = m_modules[i];
      actualStates[i] = new SwerveModuleState(module.getDriveVelocity(), Rotation2d.fromRadians(module.getSteerAngle()));
    }
    m_actualSpeeds = m_kinematics.toChassisSpeeds(actualStates);
  }

  @Override
  public void periodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_MS);

    for(int i=0; i<m_modules.length; i++) {
      var moduleState = m_states[i];
      m_modules[i].set(
        moduleState.speedMetersPerSecond / MAX_VELOCITY_MS * MAX_VOLTAGE,
        moduleState.angle.getRadians()
      );
    }

    updateActualSpeeds();
    m_odometry.update(getGyroscopeRotation(), getModulePositions());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("angleGyro", () -> getGyroscopeRotation().getDegrees(), null);
      builder.addBooleanProperty("can use Navx fused heading", () -> m_canUseFusedHeading, null);
      builder.addDoubleProperty("actual vx (ms)", () -> m_actualSpeeds.vxMetersPerSecond, null);
      builder.addDoubleProperty("actual vy (ms)", () -> m_actualSpeeds.vyMetersPerSecond, null);
      builder.addDoubleProperty("actual omega (degs)", () -> Math.toDegrees(m_actualSpeeds.omegaRadiansPerSecond), null);
  }
}

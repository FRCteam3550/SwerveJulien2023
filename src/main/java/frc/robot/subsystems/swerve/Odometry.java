package frc.robot.subsystems.swerve;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * Gère tous les aspects de l'odométrie pour la base Swerve.
 */
public class Odometry implements Sendable {
    private static double LATENCY_COMPENSATION_INCREMENT = 0.001; // 1ms
    private static final Pose2d START_POSITION =  new Pose2d(2, 2, Rotation2d.fromDegrees(0));
    private static final AprilTag TAG0 = new AprilTag(
        0,
        new Pose3d(1.73, 4.4, 0.8, new Rotation3d(0, 0, Math.toRadians(-90)))
    );
    private static final AprilTag TAG1 = new AprilTag(
        1,
        new Pose3d(0.70, 4.4, 0.8, new Rotation3d(0, 0, Math.toRadians(-90)))
    );
    private static final AprilTagFieldLayout FIELD_LAYOUT = new AprilTagFieldLayout(
        List.of(TAG0, TAG1),
        4.5,
        4.4
    );

    private final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.36, 0, 0.335),
        new Rotation3d(0,0,0)
    );

    private final PhotonPoseEstimator m_cameraPoseEstimator = new PhotonPoseEstimator(
        FIELD_LAYOUT,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        new PhotonCamera("Microsoft_LifeCam_HD-3000"),
        ROBOT_TO_CAMERA
    );
    private final Field2d m_fieldTracker;
    private final SwerveDrivePoseEstimator m_swervePoseEstimator;
    private Pose2d m_lastEstimation = new Pose2d();
    private double m_lastCameraDiffM = 0;
    private Timer m_lastCameraEstimateTime = new Timer();
    private boolean m_enableCameraEstimation = false;
    private double m_latency_compensation = -0.020; // 1ms

    public Odometry(
        Rotation2d gyroRotation,
        SwerveModulePosition[] modulePositions,
        SwerveDriveKinematics kinematics,
        Field2d fieldTracker) {

        m_fieldTracker = fieldTracker;
        m_swervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroRotation, modulePositions, START_POSITION);
        SendableRegistry.add(this, "Odometrie");
        SmartDashboard.putData(this);

        m_lastCameraEstimateTime.reset();
        m_lastCameraEstimateTime.start();

        m_fieldTracker.getObject("aprilTag0").setPose(TAG0.pose.toPose2d());
        m_fieldTracker.getObject("aprilTag1").setPose(TAG1.pose.toPose2d());
    }

    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        m_swervePoseEstimator.update(
            gyroRotation,
            modulePositions
        );

        var estimateWithoutCamera = m_swervePoseEstimator.getEstimatedPosition();

        if (m_enableCameraEstimation) {
            m_cameraPoseEstimator.setReferencePose(estimateWithoutCamera);
            var maybeEstimate = m_cameraPoseEstimator.update();

            if (maybeEstimate.isPresent() && maybeEstimate.get().estimatedPose != null) {
                var cameraEstimate = maybeEstimate.get();
                m_swervePoseEstimator.addVisionMeasurement(cameraEstimate.estimatedPose.toPose2d(), cameraEstimate.timestampSeconds + m_latency_compensation);

                m_lastCameraEstimateTime.reset();
                var estimateWithCamera = m_swervePoseEstimator.getEstimatedPosition();
                var diff = estimateWithCamera.minus(estimateWithoutCamera);
                m_lastCameraDiffM = Math.sqrt(diff.getX()*diff.getX() + diff.getY()*diff.getY());
            }

            m_lastEstimation = m_swervePoseEstimator.getEstimatedPosition();
        } else {
            m_lastEstimation = estimateWithoutCamera;
        }

        m_fieldTracker.setRobotPose(m_lastEstimation);
    }

    public void activateCameraEstimation() {
        m_enableCameraEstimation = true;
        System.out.println("Camera activee.");
    }


    public void deactivateCameraEstimation() {
        m_enableCameraEstimation = false;
    }

    public void incrementLatencyCompensation() {
        m_latency_compensation += LATENCY_COMPENSATION_INCREMENT;
    }

    public void decrementLatencyCompensation() {
        m_latency_compensation -= LATENCY_COMPENSATION_INCREMENT;
    }

    public Pose2d getPoseM() {
        m_lastEstimation = m_swervePoseEstimator.getEstimatedPosition();
        return m_lastEstimation;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getClass().getSimpleName());

        builder.addDoubleProperty("xMetres", () -> this.m_lastEstimation.getX(), null);
        builder.addDoubleProperty("yMetres", () -> this.m_lastEstimation.getY(), null);
        builder.addDoubleProperty("angleDegres", () -> m_lastEstimation.getRotation().getDegrees(), null);
        builder.addDoubleProperty("diffDistance", () -> this.m_lastCameraDiffM, null);
        builder.addBooleanProperty("positionSecuritaire", () -> this.m_lastCameraEstimateTime.get() < 10, null);
        builder.addBooleanProperty("estimation camera activee", () -> this.m_enableCameraEstimation, null);
        builder.addDoubleProperty("compensation latence", () -> this.m_latency_compensation, null);
    }
}
 
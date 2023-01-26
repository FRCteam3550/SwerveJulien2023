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

public class Odometry implements Sendable {
    private final Field2d m_field = new Field2d();
    private final AprilTagFieldLayout m_fieldLayout = new AprilTagFieldLayout(List.of(
        new AprilTag(0, new Pose3d(2, 4.4, 0.8, new Rotation3d(0, 0, Math.toRadians(-90))))),
        4.5,
        4.4
    );
    //private final AprilTagFieldLayout dispositionTerrain = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    private final PhotonCamera m_camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    private final Transform3d m_robotToCamera = new Transform3d(
        new Translation3d(0.5, 0.0, 0.2),
        new Rotation3d(0,0,0)
    ); // TODO: à mesurer
    private final PhotonPoseEstimator m_cameraPoseEstimator = new PhotonPoseEstimator(
        m_fieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        m_camera,
        m_robotToCamera
    );
    private final SwerveDrivePoseEstimator m_swervePoseEstimator;
    private Pose2d m_lastCameraEstimate = new Pose2d();
    private double m_lastCameraEstimateDistance = 0;
    private Timer m_lastCameraEstimateTime = new Timer();

    public Odometry(
        Rotation2d gyroRotation,
        SwerveModulePosition[] modulePositions,
        Pose2d startPosition,
        SwerveDriveKinematics kinematics) {

        m_swervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroRotation, modulePositions, startPosition);
        SendableRegistry.add(this, "Odometrie");
        SmartDashboard.putData(this);
        SmartDashboard.putData(m_field);

        m_lastCameraEstimateTime.reset();
        m_lastCameraEstimateTime.start();
    }

    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        m_swervePoseEstimator.update(
            gyroRotation,
            modulePositions
        );
        var estimateWithoutCamera = m_swervePoseEstimator.getEstimatedPosition();

        m_cameraPoseEstimator.setReferencePose(m_swervePoseEstimator.getEstimatedPosition());
        var maybeEstimate = m_cameraPoseEstimator.update();
        if (maybeEstimate.isPresent() && maybeEstimate.get().estimatedPose != null) {
            var cameraEstimate = maybeEstimate.get();
            m_swervePoseEstimator.addVisionMeasurement(cameraEstimate.estimatedPose.toPose2d(), cameraEstimate.timestampSeconds);

            m_lastCameraEstimateTime.reset();
            m_lastCameraEstimateTime.start();
            var estimateWithCamera = m_swervePoseEstimator.getEstimatedPosition();
            var diff = estimateWithCamera.minus(estimateWithoutCamera);
            m_lastCameraEstimateDistance = Math.sqrt(diff.getX()*diff.getX() + diff.getY()*diff.getY());
        }

        m_lastCameraEstimate = m_swervePoseEstimator.getEstimatedPosition();
        m_field.setRobotPose(m_lastCameraEstimate);
    }

    public Pose2d getPoseMeters() {
        m_lastCameraEstimate = m_swervePoseEstimator.getEstimatedPosition();
        return m_lastCameraEstimate;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getClass().getSimpleName());
        builder.addDoubleProperty("xMetres", () -> this.m_lastCameraEstimate.getX(), null);
        builder.addDoubleProperty("yMetres", () -> this.m_lastCameraEstimate.getY(), null);
        builder.addDoubleProperty("angleDegres", () -> m_lastCameraEstimate.getRotation().getDegrees(), null);
        builder.addDoubleProperty("diffDistance", () -> this.m_lastCameraEstimateDistance, null);
        builder.addBooleanProperty("positionSecuritaire", () -> this.m_lastCameraEstimateTime.get() < 10, null);
    }
}
 
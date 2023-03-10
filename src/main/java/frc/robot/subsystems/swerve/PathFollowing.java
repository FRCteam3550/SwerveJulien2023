package frc.robot.subsystems.swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathFollowing {
    private static final double MAX_VELOCITY_MS = 6.28; // TODO: mesurer
    private static final double MAX_ACCELERATION_MS2 = 3.14; // TODO: mesurer
    private static final Pose2d ZERO_POSE = new Pose2d();

    private static final double K_P_TRANS = 6;
    private static final double K_I_TRANS = 0;
    private static final double K_D_TRANS = 0;

    private static final double K_P_ROT = 4;
    private static final double K_I_ROT = 0;
    private static final double K_D_ROT = 0;

    private final Field2d m_fieldTracker;
    private final PIDController m_xController = new PIDController(K_P_TRANS, K_I_TRANS, K_D_TRANS);
    private final PIDController m_yController = new PIDController(K_P_TRANS, K_I_TRANS, K_D_TRANS);
    private final ProfiledPIDController m_theta_controller = new ProfiledPIDController(
        K_P_ROT, K_I_ROT, K_D_ROT,
        new TrapezoidProfile.Constraints(MAX_VELOCITY_MS, MAX_ACCELERATION_MS2)
    );
    private final HolonomicDriveControllerWithTelemetry m_controller = new HolonomicDriveControllerWithTelemetry(m_xController, m_yController, m_theta_controller);
    private final SwerveDriveKinematics m_kinematics;
    private final TrajectoryConfig m_trajectoryConfig;
    private final Chassis m_chassis;

    public PathFollowing(Chassis chassis, SwerveDriveKinematics kinematics, Field2d fieldTracker) {
        m_kinematics = kinematics;
        m_fieldTracker = fieldTracker;
        m_chassis = chassis;
        m_trajectoryConfig = new TrajectoryConfig( MAX_VELOCITY_MS, MAX_ACCELERATION_MS2).setKinematics(m_kinematics);
        m_theta_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Emm??ne le robot ?? la destination donn??es, dans le r??f??rentiel du terrain.
     * 
     * Le robot va en parall??le du d??placement se mettre dans la bonne orientation le plus rapidement possible au d??but du d??placement.
     * Voir note 2: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.html#%3Cinit%3E(edu.wpi.first.math.trajectory.Trajectory,java.util.function.Supplier,edu.wpi.first.math.kinematics.SwerveDriveKinematics,edu.wpi.first.math.controller.HolonomicDriveController,java.util.function.Consumer,edu.wpi.first.wpilibj2.command.Subsystem...)
     * @param destination une position du terrain, dans le r??f??rentiel du terrain.
     */
    public Command goTo(Pose2d destination) {
        return m_chassis.runOnce(() -> {
            var start = m_chassis.odometryEstimation();

            var fieldRelativeTrajectory = TrajectoryGenerator.generateTrajectory(
                start,  // On part de l?? o?? on est
                List.of(), // Aucun point interm??diaire: on va directement ?? la destination
                destination,
                m_trajectoryConfig
            );

            follow(fieldRelativeTrajectory);
        });
    }

    /**
     * D??place le robot sur une trajectoire dans le r??f??rentiel du robot. Le robot part donc de l?? o?? il est.
     * 
     * Le robot va en parall??le du d??placement se mettre dans la bonne orientation le plus rapidement possible au d??but du d??placement.
     * Voir note 2: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.html#%3Cinit%3E(edu.wpi.first.math.trajectory.Trajectory,java.util.function.Supplier,edu.wpi.first.math.kinematics.SwerveDriveKinematics,edu.wpi.first.math.controller.HolonomicDriveController,java.util.function.Consumer,edu.wpi.first.wpilibj2.command.Subsystem...)
     * @param points Points ?? suivre, dans le r??f??rentiel du robot.
     * @param finalPose Position d'arriv??e, toujours dans le r??f??rentiel du robot.
     */
    public Command robotRelativeMove(List<Translation2d> points, Pose2d finalPose) {
        return m_chassis.runOnce(() -> {
            var robotRelativeTrajectory = TrajectoryGenerator.generateTrajectory(
                ZERO_POSE, // Dans le r??f??rentiel du robot, le point de d??part, c??d l?? o?? es le robot, est par d??finition ?? (0, 0)
                points,
                finalPose,
                m_trajectoryConfig
            );
            var robotToField = m_chassis.odometryEstimation().minus(ZERO_POSE);
            var fieldRelativeTrajectory = robotRelativeTrajectory.transformBy(robotToField);

            follow(fieldRelativeTrajectory);
        });
    }

    private void follow(Trajectory trajectoire) {        
        m_fieldTracker.getObject("trajectoire").setTrajectory(trajectoire);
        m_controller.reset();

        var swerveControllerCommand = new SwerveControllerCommandWithTelemetry(
            trajectoire,
            m_chassis::odometryEstimation,
            m_kinematics,
            m_controller,
            m_chassis::setModuleStates,
            m_chassis
        );

        swerveControllerCommand
            .andThen(m_chassis::stopMotors)
            .withName("trajectoireAuto")
            .schedule();;
    }        
}

package frc.robot.subsystems.swerve;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PathFollowing {
    private static final double MAX_VELOCITY_MS = 6.28; // TODO: mesurer
    private static final double MAX_ACCELERATION_MS2 = 3.14; // TODO: mesurer
    private static final Pose2d ZERO_POSE = new Pose2d();

    private final Field2d m_fieldTracker;
    private final ProfiledPIDController theta_controller = new ProfiledPIDController(
        1, 0, 0,
        new TrapezoidProfile.Constraints(MAX_VELOCITY_MS, MAX_ACCELERATION_MS2)
    );
    private final HolonomicDriveController m_controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), // x controller
        new PIDController(1, 0, 0), // y controller
        theta_controller
    );
    private final SwerveDriveKinematics m_kinematics;
    private final TrajectoryConfig m_trajectoryConfig;
    private final Chassis m_chassis;

    public PathFollowing(Chassis chassis, SwerveDriveKinematics kinematics, Field2d fieldTracker) {
        m_kinematics = kinematics;
        m_fieldTracker = fieldTracker;
        m_chassis = chassis;
        m_trajectoryConfig = new TrajectoryConfig( MAX_VELOCITY_MS, MAX_ACCELERATION_MS2).setKinematics(m_kinematics);
        theta_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Emmène le robot à la destination données, dans le référentiel du terrain.
     * @param destination une position du terrain, dans le référentiel du terrain.
     */
    public Command goTo(Pose2d destination) {
        var fieldRelativeTrajectory = TrajectoryGenerator.generateTrajectory(
            m_chassis.odometryEstimation(),  // On part de là où on est
            List.of(), // Aucun point intermédiaire: on va directement à la destination
            destination,
            m_trajectoryConfig
        );

        return follow(fieldRelativeTrajectory);
    }

    /**
     * Déplace le robot sur une trajectoire dans le référentiel du robot. Le robot part donc de là où il est.
     * @param points Points à suivre, dans le référentiel du robot.
     * @param finalPose Position d'arrivée, toujours dans le référentiel du robot.
     */
    public Command robotRelativeMove(List<Translation2d> points, Pose2d finalPose) {
        var robotRelativeTrajectory = TrajectoryGenerator.generateTrajectory(
            ZERO_POSE, // Dans le référentiel du robot, le point de départ, càd là où es le robot, est par définition à (0, 0)
            points,
            finalPose,
            m_trajectoryConfig
        );
        var robotToField = m_chassis.odometryEstimation().minus(ZERO_POSE);
        var fieldRelativeTrajectory = robotRelativeTrajectory.transformBy(robotToField);

        return follow(fieldRelativeTrajectory);
    }

    private Command follow(Trajectory trajectoire) {        
        return m_chassis.runOnce(() -> {
            m_fieldTracker.getObject("trajectoire").setTrajectory(trajectoire);

            var swerveControllerCommand = new SwerveControllerCommand(
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
                .schedule();
        });
    }        
}

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Chassis extends Subsystem {
  Pose2d odometryEstimation();
  void stopMotors();
  void setModuleStates(SwerveModuleState[] states);
}

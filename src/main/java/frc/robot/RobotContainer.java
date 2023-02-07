// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotContainer {
  private final CommandXboxController m_gamepad = new CommandXboxController(0);
  private final SwerveDrivetrain m_swerveDrive = new SwerveDrivetrain(m_gamepad);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_gamepad.a().onTrue(m_swerveDrive.forward1MeterCommand());
    m_gamepad.b().onTrue(m_swerveDrive.goToFrontOfTag0());
    m_gamepad.x().onTrue(m_swerveDrive.decrementCameraLatencyCompensation());
    m_gamepad.y().onTrue(m_swerveDrive.incrementCameraLatencyCompensation());
    m_gamepad.leftBumper().onTrue(m_swerveDrive.deactivateCameraEstimation());
    m_gamepad.rightBumper().onTrue(m_swerveDrive.activateCameraEstimation());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

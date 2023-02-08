// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotContainer {
  private final XboxController m_gamepad = new XboxController(0);
  private final SwerveDrivetrain m_swerveDrive = new SwerveDrivetrain(m_gamepad);
  private final JoystickButton m_aGreenButton = new JoystickButton(m_gamepad, Button.kA.value);
  private final JoystickButton m_bRedButton = new JoystickButton(m_gamepad, Button.kB.value);
  private final JoystickButton m_xButton = new JoystickButton(m_gamepad, Button.kX.value);
  private final JoystickButton m_yButton = new JoystickButton(m_gamepad, Button.kY.value);
  private final JoystickButton m_leftRButton = new JoystickButton(m_gamepad, Button.kLeftBumper.value);
  private final JoystickButton m_rightRButton = new JoystickButton(m_gamepad, Button.kRightBumper.value);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_aGreenButton.onTrue(m_swerveDrive.forward1MeterCommand());
    m_bRedButton.onTrue(m_swerveDrive.goToFrontOfTag0());
    m_leftRButton.onTrue(m_swerveDrive.deactivateCameraEstimation());
    m_rightRButton.onTrue(m_swerveDrive.activateCameraEstimation());
    m_xButton.onTrue(m_swerveDrive.decrementCameraLatencyCompensation());
    m_yButton.onTrue(m_swerveDrive.incrementCameraLatencyCompensation());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

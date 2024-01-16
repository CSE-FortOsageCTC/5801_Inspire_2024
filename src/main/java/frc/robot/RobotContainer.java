// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driver = new Joystick(0);

  private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton climbExtension = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton climbRetraction = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
 

  private IntakeSubsystem intakeSubsystem;
  private ClimbingSubsystem climbingSubsystem;
  private Swerve s_Swerve;

  public RobotContainer() {
    intakeSubsystem = IntakeSubsystem.getInstance();
    climbingSubsystem = ClimbingSubsystem.getInstance();
    s_Swerve = Swerve.getInstance();
    configureBindings();
  }


  private void configureBindings() {
    intake.whileTrue(new IntakeCommand());
    climbExtension.whileTrue(new ClimbExtensionCommand());
    climbRetraction.whileTrue(new ClimbRetractionCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

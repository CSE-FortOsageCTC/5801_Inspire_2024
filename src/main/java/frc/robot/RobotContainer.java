// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.function.DoubleSupplier;

import javax.swing.JOptionPane;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;



import frc.robot.commands.AutoPickupNote;
import frc.robot.commands.DefaultTeleop;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NaviToPos;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinKickerCommand;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DefaultTeleopSub;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;


public class RobotContainer {

  
  // The robot's subsystems and commands are defined here...


  // Sendable Chooser for autos
  private SendableChooser<Command> autoChooser;

  // Path planner paths
  private PathPlannerPath sixPiecePath;
  private PathPlannerPath fourPiecePathLeft;


  /* Drive Controls */

  private DefaultTeleopSub s_DefaultTeleopSub = DefaultTeleopSub.getInstance();
  private Swerve s_Swerve = Swerve.getInstance();
  private final Joystick driver = new Joystick(0);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int throttle = XboxController.Axis.kRightTrigger.value;


  /* Driver Buttons */
  private final JoystickButton elevatorUpButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton elevatorDownButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton spinKicker = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton autoAlignAmp = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton autoPickupNoteButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton naviToPos = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton shootButton = new JoystickButton(driver, XboxController.Button.kA.value);

 

  private IntakeSubsystem intakeSubsystem;
  private ClimbingSubsystem climbingSubsystem;
  private ShootCommand shootCommand;
  private IntakeCommand intakeCommand;


  
 
  public RobotContainer() {
    intakeSubsystem = IntakeSubsystem.getInstance();
    climbingSubsystem = ClimbingSubsystem.getInstance();

    sixPiecePath = PathPlannerPath.fromPathFile("6 piece path");
    fourPiecePathLeft = PathPlannerPath.fromPathFile("4 piece path left");

    NamedCommands.registerCommand("Shoot", shootCommand);
    NamedCommands.registerCommand("Intake", intakeCommand);

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("4 piece path left", AutoBuilder.followPath(sixPiecePath));
    autoChooser.addOption("6 piece path", AutoBuilder.followPath(fourPiecePathLeft));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    s_Swerve.setHeading(Rotation2d.fromDegrees(180));

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureBindings() {
    shootButton.whileTrue(new ShootCommand());
    elevatorUpButton.whileTrue(new ElevatorCommand(300)); //setpoint is subject to change.
    elevatorDownButton.whileTrue(new ElevatorCommand(100)); //setpoint is subject to change
    spinKicker.whileTrue(new SpinKickerCommand());
    yButton.whileTrue(new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(180))));
    s_DefaultTeleopSub.setDefaultCommand(new DefaultTeleop(driver, translationAxis, strafeAxis, rotationAxis, true, throttle));
    naviToPos.whileTrue(new NaviToPos(0.0,0.0,0.0));
    autoPickupNoteButton.whileTrue((new AutoPickupNote()));
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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


import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DefaultTeleopSub;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.DefaultTeleop;

public class RobotContainer {


  // Sendable Chooser for autos
  private SendableChooser<String> autoChooser;

  // Path planner paths
  private PathPlannerPath sixPiecePath;
  private PathPlannerPath fourPiecePathLeft;
  private PathPlannerPath threePiecePathMB;
  private PathPlannerPath blueCenterScorePath;
  private PathPlannerPath blueTopStartPath;
  private PathPlannerPath blueTopScorePath;
  private PathPlannerPath blueFinishCentralPath;
  private PathPlannerPath rotatePath;
  private PathPlannerPath sevenPiecePath;
  private PathPlannerPath fourPieceNoTeamPath;


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
  private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton climbExtension = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton climbRetraction = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
  private final JoystickButton autoAlignAmp = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton autoAlignShooterSpeaker = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton autoAlignNote = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);

  private IntakeSubsystem intakeSubsystem;
  private ClimbingSubsystem climbingSubsystem;
  private ShootCommand shootCommand;
  private IntakeCommand intakeCommand;

  
 
  public RobotContainer() {
    intakeSubsystem = IntakeSubsystem.getInstance();
    climbingSubsystem = ClimbingSubsystem.getInstance();
    shootCommand = new ShootCommand();

    //Register Named Commands
    //NamedCommands.registerCommand("Shoot", shootCommand);
    //NamedCommands.registerCommand("Intake", intakeCommand);

    //Set up PathPlannerPaths
    sixPiecePath = PathPlannerPath.fromPathFile("6 piece path");
    fourPiecePathLeft = PathPlannerPath.fromPathFile("4 piece path left");
    threePiecePathMB = PathPlannerPath.fromPathFile("3 piece by MB path");
    blueCenterScorePath = PathPlannerPath.fromPathFile("Blue Center Score");
    blueTopStartPath = PathPlannerPath.fromPathFile("Blue Top Start");
    blueTopScorePath = PathPlannerPath.fromPathFile("Blue Top Score");
    blueFinishCentralPath = PathPlannerPath.fromPathFile("Blue Finish Central");
    rotatePath = PathPlannerPath.fromPathFile("Rotate");
    sevenPiecePath = PathPlannerPath.fromPathFile("7 piece path");
    fourPieceNoTeamPath = PathPlannerPath.fromPathFile("4 piece auto no team path");




    //Build, Update, and Close the autoChooser
    autoChooser = new SendableChooser<>();
    //autoChooser = AutoBuilder.buildAutoChooser("3-Piece Auto");
    autoChooser.setDefaultOption("4 piece path left", "4 piece path left");
    autoChooser.addOption("6 piece path", "6 piece path");
    autoChooser.addOption("3 piece auto James", "3-Piece Auto");
    autoChooser.addOption("3 piece auto Matthew", "3 piece pickup by MB");
    autoChooser.addOption("Blue Top Score", "Blue Top Score");
    autoChooser.addOption("Rotate", "Rotate");
    autoChooser.addOption("7 piece path", "7 piece path");
    autoChooser.addOption("4 piece no team path", "4 piece auto no team path");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = null;
    Rotation2d rotation = Rotation2d.fromDegrees(180);
  
    switch (autoChooser.getSelected()) {
      case "4 piece path left":
        path = fourPiecePathLeft;
        break;
      case "6 piece path":
        path = sixPiecePath;
        break;
      case "3 piece pickup by MB":
        path = threePiecePathMB;
        break;
      case "Blue Top Score":
        path = blueTopScorePath;
        break;
      case "Rotate":
        path = rotatePath;
        break;
      case "7 piece path":
        path = sevenPiecePath;
        break;
      case "4 piece auto no team path":
        path = fourPieceNoTeamPath;
        break;
    }
    Pose2d startingPose = path.getStartingDifferentialPose();
    s_Swerve.setPose(startingPose);
    s_Swerve.setHeading(rotation);
    return AutoBuilder.followPath(path);
  }

  private void configureBindings() {
    shootButton.whileTrue(new ShootCommand());
    elevatorUpButton.whileTrue(new ElevatorCommand(300)); //setpoint is subject to change.
    elevatorDownButton.whileTrue(new ElevatorCommand(100)); //setpoint is subject to change
    spinKicker.whileTrue(new SpinKickerCommand());
    yButton.whileTrue(new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(180))));
    s_DefaultTeleopSub.setDefaultCommand(new DefaultTeleop(driver, translationAxis, strafeAxis, rotationAxis, true, throttle));
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;


import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.math.geometry.Rotation2d;


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
  private PathPlannerPath testAutoPath;


  /* Drive Controls */
  private DefaultTeleopSub s_DefaultTeleopSub = DefaultTeleopSub.getInstance();
  private Swerve s_Swerve = Swerve.getInstance();
  private ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();
  private ClimbingSubsystem s_ClimbingSubsystem = ClimbingSubsystem.getInstance();
  private ElevatorSubsystem s_ElevatorSubsystem = ElevatorSubsystem.getInstance();
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed


  /* Driver Buttons */

  private final JoystickButton autoAlignAmp = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton autoAlignSpeaker = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton autoAlignNote = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  

  /* Operator Buttons */
  private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Axis.kRightTrigger.value);
  private final JoystickButton elevatorUpButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton elevatorDownButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton flyWheel = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton resetClimbers = new JoystickButton(operator, XboxController.Button.kBack.value);

  // private final JoystickButton climbExtention = new JoystickButton(operator, XboxController.Button.kA.value);  change this to d-pad up
  // private final JoystickButton climbRetraction = new JoystickButton(operator, XboxController.Button.kA.value);  change this to d-pad down

  private final JoystickButton autoBalanceClimb = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final POVButton upDPad = new POVButton(operator, 0);
  private final POVButton downDPad = new POVButton(operator, 180);

 
  public RobotContainer() {
    AlignPosition.setPosition(AlignPosition.Manual);

    //Register Named Commands

    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> AlignmentTransitions.scheduleShoot()));

    NamedCommands.registerCommand("Intake", new InstantCommand(() -> AlignmentTransitions.scheduleIntake()));

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
    testAutoPath = PathPlannerPath.fromPathFile("Test Auto");




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
    autoChooser.addOption("Test Auto", "Test Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = null;

    //Rotation2d rotation = new Rotation2d(-57.6);
    // s_Swerve.setHeading(rotation);

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
      case "Test Auto":
        path = testAutoPath;
        break;
    }

    path = DriverStation.getAlliance().get().equals(Alliance.Red) ? path.flipPath() : path;

    Pose2d startingPose = path.getStartingDifferentialPose();
    s_Swerve.setPose(startingPose);
    s_Swerve.setHeading(Rotation2d.fromDegrees(180));

    return AutoBuilder.followPath(path);
  }

  private void configureBindings() {

    elevatorUpButton.whileTrue(new ElevatorCommand(0.5)); //setpoint is subject to change.
    elevatorDownButton.whileTrue(new ElevatorCommand(-0.5)); //setpoint is subject to change
    flyWheel.whileTrue(new FlyWheelCommand());
    zeroGyro.whileTrue(new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(180))));
    intakeIn.whileTrue(new IntakeInCommand());
    intakeOut.whileTrue(new IntakeOutCommand());
    autoBalanceClimb.whileTrue(new AutoBalanceClimb());
    autoAlignSpeaker.whileTrue(new InstantCommand(() -> AlignmentTransitions.transitionToSpeaker()));
    autoAlignAmp.whileTrue(new InstantCommand(() -> AlignmentTransitions.transitionToAmp()));
    autoAlignNote.whileTrue(new InstantCommand(() -> AlignmentTransitions.transitionToNote()));
    s_DefaultTeleopSub.setDefaultCommand(new DefaultTeleop(driver, operator));
    s_ShooterSubsystem.setDefaultCommand(new ShootCommand(operator));
    s_ElevatorSubsystem.setDefaultCommand(new ElevatorDefaultCommand());
    //shootButton.whileTrue(new ShootCommand(operator));
    autoBalanceClimb.whileTrue(new AutoBalanceClimb());
    resetClimbers.whileTrue(new ClimbReset(-0.75, -0.75));
    // upDPad.whileTrue(new InstantCommand(() -> climbingSubsystem.climbControl(0.5, 0.5)));
    // downDPad.whileTrue(new InstantCommand(() -> climbingSubsystem.climbControl(-0.5, -0.5)));
  
  }

}
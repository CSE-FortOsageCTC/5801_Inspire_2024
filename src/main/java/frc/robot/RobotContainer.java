// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.swing.GroupLayout.Alignment;

import org.json.simple.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Rotation2d;


public class RobotContainer {

  public ElevatorDefaultCommand elevatorDefaultCommand;
  // Sendable Chooser for autos
  private SendableChooser<String> autoChooser;

  // Path planner paths
  private PathPlannerPath redLeft4PiecePath;
  private PathPlannerPath redMid4PiecePath;
  private PathPlannerPath blueRight4PiecePath;
  private PathPlannerPath redLeft4CenterPiecePath;
  private PathPlannerPath copyMid4PiecePath;
  private PathPlannerPath mid5PiecePath;
  private PathPlannerPath workingMid5PiecePath;
  private PathPlannerPath copyMid5PiecePath;
  private PathPlannerPath farSide3PiecePath;
  private PathPlannerPath side3PiecePath;




  /* Drive Controls */
  private DefaultTeleopSub s_DefaultTeleopSub = DefaultTeleopSub.getInstance();
  private Swerve s_Swerve = Swerve.getInstance();
  private ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();
  private ClimbingSubsystem s_ClimbingSubsystem = ClimbingSubsystem.getInstance();
  private ElevatorSubsystem s_ElevatorSubsystem = ElevatorSubsystem.getInstance();
  private LEDSubsystem s_LEDSubsystem = LEDSubsystem.getInstance();
  private AmpArmSubsystem s_AmpArmSubsystem = AmpArmSubsystem.getInstance();
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
  private final JoystickButton ampFlyWheel = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton flyWheel = new JoystickButton(operator, XboxController.Button.kA.value);
  //private final JoystickButton resetClimbers = new JoystickButton(operator, XboxController.Button.kBack.value);
  private final JoystickButton yButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton ampArmButton = new JoystickButton(operator, XboxController.Button.kB.value);


  // private final JoystickButton climbExtention = new JoystickButton(operator, XboxController.Button.kA.value);  change this to d-pad up
  // private final JoystickButton climbRetraction = new JoystickButton(operator, XboxController.Button.kA.value);  change this to d-pad down

  private final JoystickButton autoBalanceClimb = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final POVButton climbersUp = new POVButton(operator, 0);
  private final POVButton climbersDown = new POVButton(operator, 180);
 
  public RobotContainer() {
    AlignPosition.setPosition(AlignPosition.Manual);

    elevatorDefaultCommand = new ElevatorDefaultCommand(operator);

    //Register Named Commands
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> AlignmentTransitions.scheduleShoot()));

    NamedCommands.registerCommand("Intake", new InstantCommand(() -> AlignmentTransitions.scheduleIntake()));

    NamedCommands.registerCommand("IntakeOnly", new InstantCommand(() -> AlignmentTransitions.scheduleOnlyIntake()));
    

    //Set up PathPlannerPaths
    redLeft4PiecePath = PathPlannerPath.fromPathFile("RED LEFT 4 piece path");
    redMid4PiecePath = PathPlannerPath.fromPathFile("RED MID 4 piece path");
    copyMid4PiecePath = PathPlannerPath.fromPathFile("Copy of MID 4 piece path");
    blueRight4PiecePath = PathPlannerPath.fromPathFile("BLUE RIGHT 4 piece path");
    redLeft4CenterPiecePath = PathPlannerPath.fromPathFile("RED LEFT 4 center piece path");
    farSide3PiecePath = PathPlannerPath.fromPathFile("Far SIDE 3 piece path");
    side3PiecePath = PathPlannerPath.fromPathFile("SIDE 3 piece path");
    mid5PiecePath = PathPlannerPath.fromPathFile("MID 5 piece path");
    workingMid5PiecePath = PathPlannerPath.fromPathFile("Working MID 5 piece path");
    copyMid5PiecePath = PathPlannerPath.fromPathFile("Copy of Working MID 5 piece path");

  
  




    //Build, Update, and Close the autoChooser
    autoChooser = new SendableChooser<>();

    
    autoChooser.setDefaultOption("MID 5 piece", "MID 5 piece auto");
    autoChooser.addOption("Far SIDE 3 piece", "Far SIDE 3 piece auto");
    autoChooser.addOption("SIDE 3 piece", "SIDE 3 piece auto");
    autoChooser.addOption("MID 4 piece", "MID 4 piece auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  public Pose2d getStartingPosition() {
    PathPlannerPath path = null;
    
    switch (autoChooser.getSelected()) {
      case "MID 4 piece auto":
        path = redMid4PiecePath;
        break;
      case "Far SIDE 3 piece auto":
        path = farSide3PiecePath;
        break;
      case "SIDE 3 piece auto":
        path = side3PiecePath;
        break;
      case "MID 5 piece auto":
        path = mid5PiecePath;
        break;
    }
    
    path = DriverStation.getAlliance().get().equals(Alliance.Red) ? path.flipPath() : path;

    return path.getPreviewStartingHolonomicPose();
  } 

  public Swerve getSwerve() {
    return s_Swerve;
  }

  public LEDSubsystem getLEDSub() {
    return s_LEDSubsystem;
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = null;
    String auto = null;
    //Rotation2d rotation = new Rotation2d(-57.6);
    // s_Swerve.setHeading(rotation);
    switch (autoChooser.getSelected()) {
      case "MID 4 piece auto":
        auto = "MID 4 piece auto";
        path = redMid4PiecePath;
        break;
      case "Far SIDE 3 piece auto":
        auto = "Far SIDE 3 piece auto";
        path = farSide3PiecePath;
        break;
      case "SIDE 3 piece auto":
        auto = "SIDE 3 piece auto";
        path = side3PiecePath;
        break;
      case "MID 5 piece auto":
        auto = "MID 5 piece auto";
        path = mid5PiecePath;
        break;
    }

    path = DriverStation.getAlliance().get().equals(Alliance.Red) ? path.flipPath() : path;
    // SmartDashboard.putNumber("Ending Point X", path.getPoint(1).position.getX());
    // SmartDashboard.putNumber("Ending Point Y", path.getPoint(1).position.getY());
   
    // SmartDashboard.putNumber("Starting Point X", path.getPoint(0).position.getX());
    // SmartDashboard.putNumber("Starting Point Y", path.getPoint(0).position.getY());
    // Pose2d startingPose = AutoBuilder.getStartingPoseFromJson();//path.getPreviewStartingHolonomicPose();
    // s_Swerve.setPose(startingPose);

    //s_Swerve.setHeading(Rotation2d.fromDegrees(0));
    // AlignPosition.setPosition(AlignPosition.SpeakerPos);
    // Pose2d startPose = PathPlannerAuto.getStaringPoseFromAutoFile(auto);
    s_Swerve.setHeading(path.getPreviewStartingHolonomicPose().getRotation());
    return AutoBuilder.buildAuto(auto);

    // return AutoBuilder.followPath(path);
    
  }

  private void configureBindings() {

    // elevatorUpButton.whileTrue(new ElevatorCommand(0.5)); //setpoint is subject to change.
    // elevatorDownButton.whileTrue(new ElevatorCommand(-0.5)); //setpoint is subject to change
    elevatorDownButton.whileTrue(new InstantCommand(() -> elevatorDefaultCommand.decrement()));
    elevatorUpButton.whileTrue(new InstantCommand(() -> elevatorDefaultCommand.increment()));
    flyWheel.whileTrue(new FlyWheelCommand(-1));
    ampFlyWheel.whileTrue(new FlyWheelCommand(-.35));                                                // 21.76844 degrees
    zeroGyro.onTrue(new InstantCommand(() -> AlignmentTransitions.zeroHeading()));
    intakeIn.whileTrue(new IntakeInCommand());
    intakeOut.whileTrue(new IntakeOutCommand());
    autoBalanceClimb.whileTrue(new AutoBalanceClimb());
    autoAlignSpeaker.onTrue(new InstantCommand(() -> AlignmentTransitions.transitionToSpeaker()));
    autoAlignAmp.whileTrue(new InstantCommand(() -> AlignmentTransitions.transitionToAmp()));
    autoAlignNote.onTrue(new InstantCommand(() -> AlignmentTransitions.transitionToNote()));
    yButton.whileTrue(new FixIntakeCommand());
    s_Swerve.setDefaultCommand(new DefaultTeleop(driver, operator));
    s_ShooterSubsystem.setDefaultCommand(new ShootCommand(operator));
    s_ElevatorSubsystem.setDefaultCommand(elevatorDefaultCommand);
    s_AmpArmSubsystem.setDefaultCommand(new AmpArmCommand(operator));
    //shootButton.whileTrue(new ShootCommand(operator));
    //autoBalanceClimb.whileTrue(new AutoBalanceClimb());
    //resetClimbers.whileTrue(new ClimbReset(-1, -1));
    climbersUp.whileTrue(new Climb(1, 1));
    climbersDown.whileTrue(new ClimbReset(driver));

  
  }

}
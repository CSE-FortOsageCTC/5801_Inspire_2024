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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.choreo.lib.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {

  public ElevatorDefaultCommand elevatorDefaultCommand;

  // Sendable Chooser for autos
  private SendableChooser<String> autoChooser;

  // Initialize Autonomous Choreo Paths
  ChoreoTrajectory choreoTestPath;

  // Choreo Auto Commands
  
  // NamedCommands.registerCommand("Shoot", new InstantCommand(() -> AlignmentTransitions.scheduleShoot()));

  private Swerve s_Swerve = Swerve.getInstance();
  private ShooterSubsystem s_ShooterSubsystem = ShooterSubsystem.getInstance();
  private ElevatorSubsystem s_ElevatorSubsystem = ElevatorSubsystem.getInstance();
  private LEDSubsystem s_LEDSubsystem = LEDSubsystem.getInstance();
  private AmpArmSubsystem s_AmpArmSubsystem = AmpArmSubsystem.getInstance();
  private ChoreoSubsystem s_ChoreoSubsystem = ChoreoSubsystem.getInstance();
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final ChoreoTrajectory sevenP;
  private final ChoreoTrajectory sideAuto;
  private final ChoreoTrajectory wheelTest;
  private final ChoreoWheelTestAuto.Trajectories wheelTestTrajectories;
  private final Auto_FourP.Trajectories fourPTrajectories;
  // private final ChoreoWheelTestAuto choreoWheelTestAuto;


  /* Driver Buttons */
  private final JoystickButton autoAlignAmp = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton autoAlignSpeaker = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton autoAlignFeed = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton autoAlignNote = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);

  private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  
  private final JoystickButton elevatorUpButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton elevatorDownButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton ampFlyWheel = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton flyWheel = new JoystickButton(operator, XboxController.Button.kA.value);
  // private final JoystickButton climbExtention = new JoystickButton(operator, XboxController.Button.kA.value);  change this to d-pad up
  // private final JoystickButton climbRetraction = new JoystickButton(operator, XboxController.Button.kA.value);  change this to d-pad down
  private final JoystickButton autoBalanceClimb = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final POVButton climbersUp = new POVButton(operator, 0);

  private final POVButton climbersDown = new POVButton(operator, 180);
 
  public RobotContainer() {
    AlignPosition.setPosition(AlignPosition.Manual);

    elevatorDefaultCommand = new ElevatorDefaultCommand(operator, driver);
    /* Autonomous Setup */

    // Set Up Choreo Paths
    //choreoTestPath = Choreo.getTrajectory("Choreo1Meter");
    sevenP = Choreo.getTrajectory("SevenP");
    sideAuto = Choreo.getTrajectory("SideAuto");
    wheelTest = Choreo.getTrajectory("WheelTest");
    Choreo.getTrajectory("WheelTest2");


    wheelTestTrajectories = new ChoreoWheelTestAuto.Trajectories();
    fourPTrajectories = new Auto_FourP.Trajectories();

    //s_Swerve.setTrajectory(sevenP);

    // auto_SevenP = new Auto_SevenP(sevenP);
    // auto_Side = new Auto_Side(sideAuto);



    // Build, Update, and Close the autoChooser
    autoChooser = new SendableChooser<>();

    // Add Autonomous Routines To Dashboard Dropdown
    autoChooser.setDefaultOption("SEVEN PIECE", "SevenP");
    autoChooser.addOption("SIDE AUTO", "SideAuto");
    autoChooser.addOption("ChoreoWheelTest", "WheelTest");
    autoChooser.addOption("Blank", "Blank");
    autoChooser.addOption("FourP", "FourP");

    // Send AutoChooser To Dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  public LEDSubsystem getLEDSub() {
    return s_LEDSubsystem;
  }

  public Command getAutonomousCommand() {
    Command command = null;
    // Check Which Option Is Chosen On The Dashboard
    switch (autoChooser.getSelected()) {
      case "SevenP":
        //s_Swerve.setTrajectory(sevenP);
        command = new Auto_SevenP(sevenP);
        s_Swerve.setPose(s_ChoreoSubsystem.getFlipped() ? sevenP.getFlippedInitialPose() : sevenP.getInitialPose());
        break;
      case "WheelTest":
        command = new ChoreoWheelTestAuto(wheelTestTrajectories);
        s_Swerve.setPose(s_ChoreoSubsystem.getFlipped() ? wheelTest.getFlippedInitialPose() : wheelTest.getInitialPose());
        break;
      case "SideAuto":
        command = new Auto_Side(sideAuto);
        s_Swerve.setPose(s_ChoreoSubsystem.getFlipped() ? sideAuto.getFlippedInitialPose() : sideAuto.getInitialPose());
        break;
      case "Blank":
        command = null;
        break;
      case "FourP":
        command = new Auto_FourP(fourPTrajectories);
        s_Swerve.setPose(s_ChoreoSubsystem.getFlipped() ? fourPTrajectories.traj.getFlippedInitialPose() : fourPTrajectories.traj.getInitialPose());
        break;
    }

    // Returns Autonomous Command To Run During Auto In Robot.java
    return command;
    
  }

  private void configureBindings() {

    // elevatorUpButton.whileTrue(new ElevatorCommand(0.5)); //setpoint is subject to change.
    // elevatorDownButton.whileTrue(new ElevatorCommand(-0.5)); //setpoint is subject to change
    elevatorDownButton.onTrue(new InstantCommand(() -> s_LEDSubsystem.decrement()));
    elevatorUpButton.onTrue(new InstantCommand(() -> s_LEDSubsystem.increment()));
    flyWheel.whileTrue(new FlyWheelCommand(-1));
    ampFlyWheel.whileTrue(new FlyWheelCommand(-.35));                                                // 21.76844 degrees
    zeroGyro.onTrue(new InstantCommand(() -> AlignmentTransitions.zeroHeading()));
    intakeIn.whileTrue(new IntakeInCommand());
    intakeOut.whileTrue(new IntakeOutCommand());
    autoBalanceClimb.whileTrue(new AutoBalanceClimb());
    autoAlignSpeaker.onTrue(new InstantCommand(() -> AlignmentTransitions.transitionToSpeaker()));
    autoAlignAmp.whileTrue(new InstantCommand(() -> AlignmentTransitions.transitionToAmp()));
    autoAlignFeed.onTrue(new InstantCommand(() -> AlignmentTransitions.transitionToStage()));
    autoAlignNote.whileTrue(new AutoPickupNote());
    // yButton.whileTrue(new FixIntakeCommand());
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
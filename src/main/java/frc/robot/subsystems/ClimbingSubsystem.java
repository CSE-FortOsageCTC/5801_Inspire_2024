// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
  /** Creates a new ClimbingSubsystem. */

  private static ClimbingSubsystem climbingSubsystem;
  private static LEDSubsystem ledSubsystem;
  public static Swerve s_Swerve;
  
  public static CANSparkMax leftClimber;
  public static CANSparkMax rightClimber;


  public boolean isClimbing = false;

    public static ClimbingSubsystem getInstance(){
      if (climbingSubsystem == null){
          climbingSubsystem = new ClimbingSubsystem();
      }
      return (climbingSubsystem);
  }

  public ClimbingSubsystem(){
    s_Swerve = Swerve.getInstance();
    ledSubsystem = LEDSubsystem.getInstance();


    leftClimber = new CANSparkMax(22, MotorType.kBrushless);
    rightClimber = new CANSparkMax(23, MotorType.kBrushless);

    leftClimber.setSmartCurrentLimit(9);
    rightClimber.setSmartCurrentLimit(9);

    rightClimber.burnFlash();
    leftClimber.burnFlash();

  }

  public void climbControl(double leftSpeed, double rightSpeed){
    leftClimber.set(leftSpeed);
    rightClimber.set(rightSpeed);
    if (leftSpeed != 0 || rightSpeed != 0) {
      isClimbing = true;
    }
  }

  @Override
  public void periodic(){
    if (isClimbing) {
      ledSubsystem.setLEDs(-0.99); //"Rainbow, Rainbow Palette" 
    }
    else {
      ledSubsystem.setDefaultLEDs();
    }
    // SmartDashboard.putNumber("right Current", rightClimber.getOutputCurrent());
    // SmartDashboard.putNumber("left Current", leftClimber.getOutputCurrent());
  }
  // make a variable which takes the roll, set point proportional to the yaw
}
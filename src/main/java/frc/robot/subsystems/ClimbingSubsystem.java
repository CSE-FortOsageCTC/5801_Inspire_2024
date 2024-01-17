// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
  /** Creates a new ClimbingSubsystem. */

  private static ClimbingSubsystem climbingSubsystem;

    public static ClimbingSubsystem getInstance(){
      if (climbingSubsystem == null){
          climbingSubsystem = new ClimbingSubsystem();
      }
      return (climbingSubsystem);
  }

  public void climbingExtension(){
    System.out.println("Climbing Extension");
  }

  public void climbingRetraction(){
    System.out.println("Climbing Retraction");
  }
}

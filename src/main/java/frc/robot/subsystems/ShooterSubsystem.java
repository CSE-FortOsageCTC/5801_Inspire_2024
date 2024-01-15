package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem shooterSubsystem;
    public static ShooterSubsystem getInstance(){
        if (shooterSubsystem == null){
            shooterSubsystem = new ShooterSubsystem();
        } 
        return (shooterSubsystem);
    }
    private ShooterSubsystem(){
        
    }
    public void shoot(){
        System.out.print("shoot");
    }
    public void shootWithVelocity(String velocity){
        System.out.print("Shoot with velocity of " + velocity);
    }
}

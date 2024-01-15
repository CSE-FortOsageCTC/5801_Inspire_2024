package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private static IntakeSubsystem intakeSubsystem;

    public static IntakeSubsystem getInstance(){
        if (intakeSubsystem == null){
            intakeSubsystem = new IntakeSubsystem();
        }
        return (intakeSubsystem);
    }

    private IntakeSubsystem(){

    }

    public void intakeIn(){
        System.out.print("intake in");
    } 

    public void intakeOut(){
        System.out.print("intake out");
    }
}

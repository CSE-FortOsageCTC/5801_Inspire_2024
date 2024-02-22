package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.subsystems.Swerve;


public class AlignmentTransitions extends Command{

    private void AlignmentTransitions(){}
    
    public static void scheduleShoot(){
        Swerve.getInstance().resetAutoRotateUtil();
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoShootCommand().schedule();
        
        
    }

    public static void scheduleIntake(){
        Swerve.getInstance().resetAutoRotateUtil();
        AlignPosition.setPosition(AlignPosition.AutoPickup);
        new AutoIntakeCommand().schedule();
        
    }
}
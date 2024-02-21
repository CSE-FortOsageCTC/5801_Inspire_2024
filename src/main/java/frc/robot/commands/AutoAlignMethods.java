package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AlignPosition;
import frc.robot.commands.FlyWheelCommand;

public class AutoAlignMethods extends Command{

    public void AutoAlignAmp(){
        
    }

    
    public static void scheduleShoot(){
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoShootCommand().schedule();
        
        
    }

    public static void scheduleIntake(){
        AlignPosition.setPosition(AlignPosition.AutoPickup);
        new AutoIntakeCommand().schedule();
        
    }
}
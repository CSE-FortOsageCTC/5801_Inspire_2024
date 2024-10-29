package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AlignPosition;
import frc.robot.subsystems.Swerve;


public class AlignmentTransitions extends Command{

    private void AlignmentTransitions(){}
    
    public static void scheduleShoot(){
        //Swerve.getInstance().resetAutoRotateUtil();
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoFlywheelCommand().schedule();
    }
    // public static void scheduleFlywheel(){
    //     Swerve.getInstance().resetAutoRotateUtil();
    //     AlignPosition.setPosition(AlignPosition.SpeakerPos);
    //     new FlyWheelCommand().schedule();
    // }

    public static void scheduleFlywheels() {
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoOnlyFlywheelCommand().schedule();
    }

    public static void stopFlywheels() {
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoStopFlywheelCommand().schedule();
    }

    public static void scheduleIntake(){
        // Swerve.getInstance().resetAutoRotateUtil();
        //AlignPosition.setPosition(AlignPosition.AutoPickup);
        new AutoIntakeCommand().schedule();
    }

    public static void scheduleOnlyIntake(){
        // Swerve.getInstance().resetAutoRotateUtil();
        //AlignPosition.setPosition(AlignPosition.AutoPickup);
        new AutoOnlyIntakeCommand().schedule();
    }

    public static void transitionToSpeaker() {
        Swerve.getInstance().resetAutoRotateUtil();
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
    }

    public static void transitionToAmp() {
        Swerve.getInstance().resetAutoRotateUtil();
        AlignPosition.setPosition(AlignPosition.AmpPos);
    }

    public static void transitionToStage() {
        Swerve.getInstance().resetAutoRotateUtil();
        AlignPosition.setPosition(AlignPosition.StagePos);
    }

    public static void transitionToNote() {
        Swerve.getInstance().resetAutoRotateUtil();
        //AlignPosition.setPosition(AlignPosition.AutoPickup);
    }
    public static void zeroHeading(){
        if (DriverStation.getAlliance().get().equals(Alliance.Red)){ 
            Swerve.getInstance().setHeading(Rotation2d.fromDegrees(180));
        }
        else{
            Swerve.getInstance().setHeading(Rotation2d.fromDegrees(0));
        }
    }
}
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignPosition;
import frc.robot.subsystems.Swerve;


public class AlignmentTransitions extends Command{

    /*
     * Sets the target position to the speaker and runs the flywheels and shoots
     */
    public static void scheduleShoot(){
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoFlywheelCommand().schedule();
    }

    /*
     * Sets the target position to the speaker and runs the flywheels only
     */
    public static void scheduleFlywheels() {
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoOnlyFlywheelCommand().schedule();
    }

    /*
     * Stops the flywheels 
     */
    public static void stopFlywheels() {
        AlignPosition.setPosition(AlignPosition.SpeakerPos);
        new AutoStopFlywheelCommand().schedule();
    }

    /*
     * Intakes for 1.4 seconds or until a note is detected then shoots the not
     */
    public static void scheduleIntake(){
        new AutoIntakeCommand().schedule();
    }

    /*
     * Intakes for 1.4 seconds 
     */
    public static void scheduleOnlyIntake(){
        new AutoOnlyIntakeCommand().schedule();
    }
    /*
    * 
    */
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
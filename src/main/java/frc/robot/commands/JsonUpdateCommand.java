package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoordsOutputSubsystem;

public class JsonUpdateCommand extends Command{
    

    private CoordsOutputSubsystem s_CoordsOutputSubsystem;

    private Timer timer;

    private double timeOffset;

    private int id = 0;


    public JsonUpdateCommand() {

        s_CoordsOutputSubsystem = CoordsOutputSubsystem.getInstance();

        timer = new Timer();

    }

    @Override
    public void initialize() {
        timeOffset = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        
        if (timer.hasElapsed(0.5 + timeOffset)) {

            Pose2d botPose = s_CoordsOutputSubsystem.s_Swerve.getPose();

            String coordOutput = "(" + botPose.getX() + ", " + botPose.getY() + ")";

            s_CoordsOutputSubsystem.appendJSON(id, coordOutput);
            timeOffset = Timer.getFPGATimestamp();
            id += 0.5;

            System.out.println("JSON OUTPUT: " + id + " : " + botPose);

        } else if (timer.hasElapsed(150)) {

            s_CoordsOutputSubsystem.printJSON();

        }



    }


}

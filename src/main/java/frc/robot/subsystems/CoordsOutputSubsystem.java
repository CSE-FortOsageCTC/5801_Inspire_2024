package frc.robot.subsystems;

import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoordsOutputSubsystem extends SubsystemBase{

    private JSONObject jsonOutput;

    public Swerve s_Swerve;

    private static CoordsOutputSubsystem s_CoordsOutputSubsystem;

    public static CoordsOutputSubsystem getInstance() {
        if (s_CoordsOutputSubsystem == null) {
            s_CoordsOutputSubsystem = new CoordsOutputSubsystem();
        }
        return s_CoordsOutputSubsystem;
    }



    public CoordsOutputSubsystem() {

        s_Swerve = Swerve.getInstance();
        jsonOutput = new JSONObject();

    }

    public void appendJSON(int id, Pose2d value) {

        jsonOutput.put(id, value);

    }

    public void printJSON() {

        try {
            FileWriter file = new FileWriter("D:\\JSONOutput\\output.json");
            file.write(jsonOutput.toJSONString());
            file.close();
        } catch (IOException exception) {
            exception.printStackTrace();
        }
        
        System.out.println("JSON file created: " + jsonOutput);

    }
    

}

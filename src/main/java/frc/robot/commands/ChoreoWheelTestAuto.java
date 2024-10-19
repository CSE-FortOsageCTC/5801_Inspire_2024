package frc.robot.commands;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ChoreoSubsystem;

public class ChoreoWheelTestAuto extends SequentialCommandGroup{

    
public ChoreoWheelTestAuto() {
    super(
        
        new SequentialCommandGroup(
            ChoreoSubsystem.getInstance().setupAutonomousChoreoPath(Choreo.getTrajectory("WheelTest"))
        )


    );
}

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClimbExtensionCommand extends Command{
    ClimbingSubsystem climbingSubsystem;
    public ClimbExtensionCommand(){
    climbingSubsystem = ClimbingSubsystem.getInstance();
    addRequirements(climbingSubsystem);
    }
    @Override
    public void execute(){
        climbingSubsystem.climbingExtension();
    }
}

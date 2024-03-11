package frc.robot.commands;

public class AmpCommand extends Navigate{

    @Override
    public void initialize() {
        AlignmentTransitions.transitionToAmp();
    }
    
}

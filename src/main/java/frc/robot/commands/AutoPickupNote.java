package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoPickupNote extends Command{
    public Limelight limelight;
    public Swerve swerve;
    public IntakeSubsystem intakeSubsystem;
    public Debouncer debouncer;
    public Pose2d position;
    public Rotation2d yaw;
    public double limeX;
    public double limeY;



    public AutoPickupNote(){
        swerve = Swerve.getInstance();
        limelight = Limelight.getInstance();
        debouncer = new Debouncer(.5); 
    }
    private boolean NoteSeen(){
        return limelight.hasTag();
    }

    public double newY(){
        return limeY > 0 ? 1.0 : 0;
    }
    public double newRotation(){
        if (limeX > 0){
        return yaw.getDegrees() - 1.0;}
        else if(limeX < 0){
            return yaw.getDegrees() + 1.0;}
        else{
            return 0;
        }}

    @Override
    public void execute(){
        position = swerve.getPose();
        yaw = swerve.getGyroYaw();
        limeX = limelight.getX();
        limeY = limelight.getY();

    
        
        if (debouncer.calculate(NoteSeen())){
            System.out.println("Note in view");
            Translation2d translation = new Translation2d(0, newY()).times(Constants.Swerve.maxSpeed);
            swerve.drive(translation, newRotation(), false, true);
        }
    }
   
    public void end(){
        swerve.drive(new Translation2d(0,0), 0, true, true);
    }
} 
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ElevatorCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final PIDController upPidController;
    private final PIDController downPidController;
    private double setPoint;
    public ElevatorCommand(double setPoint){
        this.setPoint = setPoint;
        this.upPidController = new PIDController(0, 0, 0);
        this.downPidController = new PIDController(0, 0, 0);
        shooterSubsystem = ShooterSubsystem.getInstance();

        upPidController.setTolerance(5);
        upPidController.setSetpoint(300);
        SmartDashboard.putNumber("Up P", 0);
        SmartDashboard.putNumber("Up I", 0);
        SmartDashboard.putNumber("Up D", 0);
        
        downPidController.setTolerance(5);
        downPidController.setSetpoint(20);
        SmartDashboard.putNumber("Down P", 0);
        SmartDashboard.putNumber("Down I", 0);
        SmartDashboard.putNumber("Down D", 0);
    }

    // public void updateSetPoint(double setPoint){
    //     this.setPoint = setPoint;
    //     setPoint = upPidController.getSetpoint();
    // }

    @Override
    public void initialize() {
        upPidController.reset();
        downPidController.reset();
    }

    @Override
    public void execute(){
        if (up == true){shooterSubsystem.setElevatorSpeed(.2);}
        else{shooterSubsystem.setElevatorSpeed(-.2);}
        
    }
    @Override
    public void end(boolean end){
        shooterSubsystem.setElevatorSpeed(0);
        upPidController.reset();
        downPidController.reset();
    }
        
    
}

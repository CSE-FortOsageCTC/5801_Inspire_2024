package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private static ElevatorSubsystem elevatorSubsystem;

    public SkyLimelight s_Limelight = SkyLimelight.getInstance();
    public Swerve s_Swerve = Swerve.getInstance();

    private CANSparkMax elevator;

    public static ElevatorSubsystem getInstance(){
        if (elevatorSubsystem == null){
            elevatorSubsystem = new ElevatorSubsystem();
        } 
        return (elevatorSubsystem);
    }    

    public ElevatorSubsystem(){
        elevator = new CANSparkMax(25, MotorType.kBrushless);
    }

    public double getElevatorValue(){
        return elevator.getEncoder().getPosition();
    }

    public void setElevatorSpeed(double speed){
        elevator.set(speed);
        SmartDashboard.putNumber("Encoder Value", elevator.getEncoder().getPosition());
    }
}

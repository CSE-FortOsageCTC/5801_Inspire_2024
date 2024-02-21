package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private static ElevatorSubsystem elevatorSubsystem;

    private CANSparkMax rightElevator;
    private CANSparkMax leftElevator;
    //private PIDController elevatorPID;

    public static ElevatorSubsystem getInstance(){
        if (elevatorSubsystem == null){
            elevatorSubsystem = new ElevatorSubsystem();
        } 
        return (elevatorSubsystem);
    }    

    public void elevatorSubsystem(){
        rightElevator = new CANSparkMax(25, MotorType.kBrushless);
        leftElevator.follow(rightElevator);

        //elevatorPID = new PIDController(0, 0, 0);
    }

    public double getElevatorValue(){
        return rightElevator.getEncoder().getPosition();
    }

    public void setElevatorSpeed(double speed){

        double elevatorValue = getElevatorValue();
        if (elevatorValue < Constants.Swerve.minElevatorValue && speed < 0){
            speed = 0;
        }

        else if (elevatorValue > Constants.Swerve.maxElevatorValue && speed > 0){
            speed = 0;
        }
        rightElevator.set(speed);

        SmartDashboard.putNumber("Encoder Value", rightElevator.getEncoder().getPosition());
    }
}

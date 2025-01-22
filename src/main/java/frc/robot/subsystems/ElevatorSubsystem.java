package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ExternalEncoderConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkLimitSwitch.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private static ElevatorSubsystem elevatorSubsystem;
    private SparkMax shooterElevator;

    private SparkMaxConfig configurations;

    private LimitSwitchConfig limitSwitchConfig;

    public boolean isAligned = false;

    public SkyLimelight s_Limelight = SkyLimelight.getInstance();
    public Swerve s_Swerve = Swerve.getInstance();

    public static ElevatorSubsystem getInstance(){
        if (elevatorSubsystem == null){
            elevatorSubsystem = new ElevatorSubsystem();
        } 
        return (elevatorSubsystem);
    }    

    public ElevatorSubsystem(){
        shooterElevator = new SparkMax(25, MotorType.kBrushless);
        
        // shooterElevator.setIdleMode();
        // shooterElevator.enableVoltageCompensation(10);

        limitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);

        configurations.idleMode(IdleMode.kBrake);
        configurations.voltageCompensation(10);
        configurations.apply(limitSwitchConfig);

        shooterElevator.configure(configurations, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getElevatorValue(){
        return shooterElevator.getEncoder().getPosition();
    }

    public void setElevatorSpeed(double speed){
        shooterElevator.set(speed);
    }

    @Override
    public void periodic() {

        if (shooterElevator.getForwardLimitSwitch().isPressed()) {
            shooterElevator.getEncoder().setPosition(0);
        } 
        //SmartDashboard.putNumber("Encoder Value", shooterElevator.getEncoder().getPosition());

    }
}

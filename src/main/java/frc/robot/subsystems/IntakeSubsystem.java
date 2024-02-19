package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private static IntakeSubsystem intakeSubsystem;

    private static CANSparkMax lowerIntake = new CANSparkMax(26, MotorType.kBrushless);
    private static TalonSRX upperIntake = new TalonSRX(27);

    private static DigitalInput intakeSensor = new DigitalInput(1);

    public static IntakeSubsystem getInstance(){
        if (intakeSubsystem == null){
            intakeSubsystem = new IntakeSubsystem();
        }
        return (intakeSubsystem);
    }

    private IntakeSubsystem(){

    }

    public void intakeIn(){
        lowerIntake.set(-1);
        upperIntake.set(TalonSRXControlMode.PercentOutput, 1);
    } 

    public void intakeOut(){
        lowerIntake.set(1);
        upperIntake.set(TalonSRXControlMode.PercentOutput, 1);
    } 

    public void intakeStop() {
        lowerIntake.set(0);
        upperIntake.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public boolean isRingDetected() {
        return !intakeSensor.get();
    }
}

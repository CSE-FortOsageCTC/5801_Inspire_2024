package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpArmSubsystem extends SubsystemBase{
    private static AmpArmSubsystem ampArmSubsystem;
    public CANSparkMax ampArmMotor;

    public static AmpArmSubsystem getInstance(){
        if(ampArmSubsystem == null){
            ampArmSubsystem = new AmpArmSubsystem();
        }
        return ampArmSubsystem;
    }

    private AmpArmSubsystem(){
        ampArmMotor = new CANSparkMax(0, MotorType.kBrushless);

        ampArmMotor.setIdleMode(IdleMode.kCoast);
        ampArmMotor.enableVoltageCompensation(10);
        ampArmMotor.setSmartCurrentLimit(20);
        ampArmMotor.burnFlash();
    }

    public void setSpeed(double speed){
        ampArmMotor.set(speed);
    }

    public double getEncoderValue(){
        return ampArmMotor.getEncoder().getPosition();
    }
}

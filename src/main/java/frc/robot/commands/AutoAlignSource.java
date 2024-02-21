package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.SkyLimelight;
import frc.robot.subsystems.Swerve;

public class AutoAlignSource extends Command{

    private PIDController yTranslationPidController;
    private PIDController xTranslationPidController;
    private PIDController rotationPidController;

    private Swerve s_Swerve;
    private SkyLimelight s_Limelight;
    private AutoRotateUtil autoUtil;

    public AutoAlignSource() {
        this.s_Limelight = SkyLimelight.getInstance();
        this.s_Swerve = Swerve.getInstance();
        this.autoUtil = new AutoRotateUtil(0);
        addRequirements(s_Swerve, s_Limelight);
        

        // creating yTranslationPidController and setting the tolerance and setpoint
        yTranslationPidController = new PIDController(0, 0, 0);
        yTranslationPidController.setTolerance(1);
        yTranslationPidController.setSetpoint(0);
        
        // creating xTranslationPidController and setting the tolerance and setpoint
        xTranslationPidController = new PIDController(0, 0, 0);
        xTranslationPidController.setTolerance(0);
        xTranslationPidController.setSetpoint(0);
        
        // creating rotationPidController and setting the tolerance and setpoint
        rotationPidController = new PIDController(0, 0, 0);
        rotationPidController.setTolerance(2);
        rotationPidController.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        SmartDashboard.putNumber("P", 0);
        SmartDashboard.putNumber("i", 0);
        SmartDashboard.putNumber("d", 0);
        //THESE VALUES WILL NEED TO BE MESSED WITH, 0 FOR NOW
    }


    @Override
    public void execute() {
        // gets value of P,I and D from smartdashboard
        // will be removed once done tweaking
        double kP = SmartDashboard.getNumber("P", 0);
        double kI = SmartDashboard.getNumber("i", 0);
        double kD = SmartDashboard.getNumber("d", 0);
        
        // sets the PID values for the PIDControllers
        yTranslationPidController.setPID(kP, kI, kD);
        xTranslationPidController.setPID(kP, kI, kD);
        //rotationPidController.setPID(rotationkP, rotationkI, rotationkD);

        double xValue = s_Limelight.getX(); //gets the limelight X Coordinate
        double areaValue = s_Limelight.getArea(); // gets the area percentage from the limelight
        //double angularValue = s_Limelight.getSkew(); 

        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);
        //SmartDashboard.putNumber("AngularValue", angularValue);
        //SmartDashboard.putNumber("Ts0", s_Limelight.getSkew0());
        //SmartDashboard.putNumber("Ts1", s_Limelight.getSkew1());
        //SmartDashboard.putNumber("Ts2", s_Limelight.getSkew2());


        // Calculates the x and y speed values for the translation movement
        double ySpeed = yTranslationPidController.calculate(xValue);
        double xSpeed = xTranslationPidController.calculate(areaValue);
        double angularSpeed = autoUtil.calculateRotationSpeed() * Constants.Swerve.maxAngularVelocity;
        

        // moves the swerve subsystem
        Translation2d translation = new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed);
        double rotation = angularSpeed * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, true);
    }

}

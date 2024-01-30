package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AutoRotateUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SkyLimelight;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoPickupNote extends Command{
    public SkyLimelight limelight;
    public Swerve swerve;
    public IntakeSubsystem intakeSubsystem;
    public Debouncer debouncer;
    public Rotation2d yaw;
    private AutoRotateUtil autoUtil;

    public PIDController yTranslationPidController, xTranslationPidController, rotationPidController;
    public double kP;
    public double kI; 
    public double kD;


    public AutoPickupNote(){
        this.autoUtil = new AutoRotateUtil(0);
        swerve = Swerve.getInstance();
        limelight = SkyLimelight.getInstance();
        debouncer = new Debouncer(.5);

        // creating yTranslationPidController and setting the toleance and setpoint
        yTranslationPidController = new PIDController(0, 0, 0);
        yTranslationPidController.setSetpoint(0);
    
        // creating xTranslationPidController and setting the toleance and setpoint
        xTranslationPidController = new PIDController(0, 0, 0);
        xTranslationPidController.setSetpoint(0);
        
        // creating rotationPidController and setting the toleance and setpoint
        rotationPidController = new PIDController(0, 0, 0);
        rotationPidController.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        SmartDashboard.putNumber("P", 0);
        SmartDashboard.putNumber("i", 0);
        SmartDashboard.putNumber("d", 0);
        //THESE VALUES WILL NEED TO BE MESSED WITH, 0 FOR NOW

        addRequirements(swerve, intakeSubsystem);
    }
    private boolean NoteSeen(){
        return limelight.hasTag();
    }
    private double RadiansToDegrees(double rads){
        return rads * (180/Math.PI);
    }
    private double FixedDistanceCalc(){
        double objectHeight = 0;
        double cameraHeight = .5;
        double yAngle = limelight.getY();
        double skewAngle = limelight.getSkew();
        return (objectHeight - cameraHeight) / RadiansToDegrees(Math.tan(yAngle + skewAngle));
    }

    private double getDistanceMeters(){
        double focalLength =  2.9272781257541 / 1000;
        double d = (14 * .0254 * focalLength)/limelight.getArea();
        return d;
    }

    @Override
    public void execute(){
        yaw = swerve.getGyroYaw();
        
        double xValue = limelight.getX(); //gets the limelight X Coordinate
        double areaValue = limelight.getArea(); // gets the area percentage from the limelight
        double distance = getDistanceMeters();
<<<<<<< HEAD
        autoUtil.updateTargetAngle(xValue);
=======
        double fixedDistance = FixedDistanceCalc();
>>>>>>> cdb75668d156dd6f8101fc40a2597f26872f04b1

        kP = SmartDashboard.getNumber("AlignP", 0);
        kI = SmartDashboard.getNumber("AlignI", 0);
        kD = SmartDashboard.getNumber("AlignD", 0);
        
        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);



        
        if (debouncer.calculate(NoteSeen())){
            System.out.println("Note in view");
            // Calculates the x and y speed values for the translation movement
            double ySpeed = yTranslationPidController.calculate(distance * RadiansToDegrees(Math.cos(xValue)));
            double xSpeed = xTranslationPidController.calculate(distance * RadiansToDegrees(Math.sin(xValue)));
            double angularSpeed = autoUtil.calculateRotationSpeed() * Constants.Swerve.maxAngularVelocity;
            

            // moves the swerve subsystem
            Translation2d translation = new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed);
            double rotation = angularSpeed * Constants.Swerve.maxAngularVelocity;
            swerve.drive(translation, rotation, false, true);
        }
        else{
            System.out.println("Note not found");
        }
    }
   
    public void end(){
        swerve.drive(new Translation2d(0,0), 0, true, true);
        xTranslationPidController.reset();
        yTranslationPidController.reset();
    }
} 
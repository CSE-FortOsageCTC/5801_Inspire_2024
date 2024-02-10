package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.AutoRotateUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.FloorLimelight;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoPickupNote extends Command{
    public FloorLimelight limelight;
    public Swerve swerve;
    public IntakeSubsystem intakeSubsystem;
    public Debouncer debouncer;
    public Rotation2d yaw;
    private AutoRotateUtil autoUtil;

    public PIDController yTranslationPidController, xTranslationPidController, rotationPidController;
    public double kP;
    public double kI; 
    public double kD;

    public SlewRateLimiter xLimiter;
    public SlewRateLimiter yLimiter;


    public AutoPickupNote(){
        autoUtil = new AutoRotateUtil(0);
        swerve = Swerve.getInstance();
        limelight = FloorLimelight.getInstance();
        debouncer = new Debouncer(.1, Debouncer.DebounceType.kBoth);
        intakeSubsystem = IntakeSubsystem.getInstance();

        // creating yTranslationPidController and setting the tolerance and setpoint
        yTranslationPidController = new PIDController(.5, 0, 0);
        yTranslationPidController.setSetpoint(0);
    
        // creating xTranslationPidController and setting the tolerance and setpoint
        xTranslationPidController = new PIDController(.003, 0, 0);
        xTranslationPidController.setSetpoint(0);

        xLimiter = new SlewRateLimiter(1);
        yLimiter = new SlewRateLimiter(1);
        
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
        double focalLength =  2.9272781257541;
        double d = (14 * .0254 * focalLength)/limelight.getArea();
        return d;
    }

    @Override
    public void execute(){
        yaw = swerve.getGyroYaw();
        boolean seen = NoteSeen();
        
        double xValue = limelight.getX(); //gets the limelight X Coordinate
        double yValue = limelight.getY();
        double areaValue = limelight.getArea(); // gets the area percentage from the limelight
        double distance = getDistanceMeters();
        autoUtil.updateTargetAngle(xValue);

        double fixedDistance = FixedDistanceCalc();
        
        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);
        SmartDashboard.putBoolean("NoteSeen", seen);
        SmartDashboard.putNumber("Distance", distance);



        
        if (debouncer.calculate(seen)){
            System.out.println("Note in view");
            // Calculates the x and y speed values for the translation movement
            double ySpeed = yTranslationPidController.calculate(distance);
            double xSpeed = xTranslationPidController.calculate(xValue);
            double angularSpeed = autoUtil.calculateRotationSpeed() * Constants.Swerve.maxAngularVelocity;
            ySpeed = MathUtil.clamp(ySpeed, -.5, .5);
            xSpeed = MathUtil.clamp(xSpeed, -1, 1);

            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);

            // xSpeed = xLimiter.calculate(xSpeed);
            // ySpeed = yLimiter.calculate(ySpeed);
            // moves the swerve subsystem
            Translation2d translation = new Translation2d(ySpeed, -xSpeed).times(Constants.Swerve.maxSpeed);
            double rotation = angularSpeed * Constants.Swerve.maxAngularVelocity;
            swerve.drive(translation, 0, false, false);
        }
        else{
            System.out.println("Note not found");
        }
    }
   
    public void end(){
        swerve.drive(new Translation2d(0,0), 0, true, true);
        xTranslationPidController.reset();
        yTranslationPidController.reset();
        autoUtil.reset();
    }
} 
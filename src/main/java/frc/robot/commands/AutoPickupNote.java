package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRotateUtil;
import edu.wpi.first.math.controller.PIDController;


import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.FloorLimelight;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class AutoPickupNote extends Command{
    public FloorLimelight limelight;
    public Swerve swerve;
    public IntakeSubsystem intakeSubsystem;
    public Debouncer debouncer;
    

    public int detectedDelayCount;

    public double xValue;
    public double yValue;
    public double areaValue;

    public Rotation2d yaw;
    private AutoRotateUtil autoUtil;

    public PIDController yTranslationPidController, xTranslationPidController;
    public double kP;
    public double kI; 
    public double kD;

    public Pose2d position;

    public boolean hasPickedUpNote=false;

    public int waitFor=0;
    public int counter=0;


    public AutoPickupNote(int waitFor){
        swerve = Swerve.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        detectedDelayCount = 0;
        autoUtil = new AutoRotateUtil(0);
        this.waitFor = waitFor;

        limelight = FloorLimelight.getInstance();

        debouncer = new Debouncer(.5);

        // creating yTranslationPidController and setting the toleance and setpoint
        yTranslationPidController = new PIDController(.2, 0, 0);
        yTranslationPidController.setTolerance(1);
        yTranslationPidController.setSetpoint(0);
        
        // creating xTranslationPidController and setting the toleance and setpoint
        xTranslationPidController = new PIDController(.2, 0, 0);
        xTranslationPidController.setTolerance(0);
        xTranslationPidController.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        // SmartDashboard.putNumber("P", 0);
        // SmartDashboard.putNumber("i", 0);
        // SmartDashboard.putNumber("d", 0);
        //THESE VALUES WILL NEED TO BE MESSED WITH, 0 FOR NOW

        addRequirements(intakeSubsystem);
    }

    public AutoPickupNote() {
        new AutoPickupNote(0);
    }

    private boolean NoteSeen(){
        return limelight.hasTag();
    }


    public double newY(){
        return limelight.getY() > 0 ? 1.0 : 0;
    }

    public double newRotation(){
        if (limelight.getX() > 0){
        return yaw.getDegrees() - 1.0;}
        else if(limelight.getX() < 0){
            return yaw.getDegrees() + 1.0;}
        else{
            return 0;
        }
    }


    @Override
    public void execute(){

        kP = SmartDashboard.getNumber("AlignP", 0);
        kI = SmartDashboard.getNumber("AlignI", 0);
        kD = SmartDashboard.getNumber("AlignD", 0);
        
        // SmartDashboard.putNumber("Xvalue", xValue);
        // SmartDashboard.putNumber("Areavalue", areaValue);
        // SmartDashboard.putNumber("AngularValue", angularValue);
        // SmartDashboard.putNumber("Ts0", limelight.getSkew0());
        // SmartDashboard.putNumber("Ts1", limelight.getSkew1());
        // SmartDashboard.putNumber("Ts2", limelight.getSkew2());

        boolean ringDetected = intakeSubsystem.isRingDetected(); //ring is detected in the robot
        if (ringDetected && hasPickedUpNote){
            detectedDelayCount++;
        }
        System.out.println(detectedDelayCount);
        counter++;
        if (debouncer.calculate(NoteSeen()) && !ringDetected && counter>waitFor){
            xValue = limelight.getX(); //gets the limelight X Coordinate
            yValue = limelight.getY();
            areaValue = limelight.getArea(); // gets the area percentage from the limelight
            SmartDashboard.putNumber("Area", areaValue);
            autoUtil.updateTargetAngle(-xValue);

            if(yValue <= -20){
                intakeSubsystem.intakeIn();
                hasPickedUpNote=true;
            }
            // System.out.println("Note in view");
            // Calculates the x and y speed values for the translation movement
            //double ySpeed = yTranslationPidController.calculate(xValue);
            double xSpeed = xTranslationPidController.calculate(100/-areaValue);

            double angularSpeed = autoUtil.calculateRotationSpeed() * Constants.Swerve.maxAngularVelocity;
            

            // moves the swerve subsystem
            Translation2d translation = new Translation2d(xSpeed, 0);
            double rotation = angularSpeed;
            SmartDashboard.putNumber("rotation speed", rotation);

            swerve.setAutoDriveParams(translation, rotation, false, true);
            swerve.readyToPickUp = true;
        } else { //TODO: check for tele-op later cuz you know... kinda important and stuff. ALSO second pickup instance not working
            //System.out.println("Note not found");
            // swerve.drive(new Translation2d(0,0), 0, true, true);
            swerve.readyToPickUp = false;
            // if (detectedDelayCount >= 5){
            //     intakeSubsystem.intakeStop();
            //     detectedDelayCount = 0;
                
            // }
        }
    }
    @Override 
    public boolean isFinished(){
        // return false;
        return detectedDelayCount >= 5;
    }

    @Override
    public void end(boolean over){
        swerve.drive(new Translation2d(0,0), 0, true, true);
        swerve.readyToPickUp = false;
        intakeSubsystem.intakeStop();
        detectedDelayCount = 0;
        xTranslationPidController.reset();
        yTranslationPidController.reset();
        autoUtil.reset();

    }
} 
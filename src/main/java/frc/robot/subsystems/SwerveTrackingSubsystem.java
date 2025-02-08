package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.Constants;

public class SwerveTrackingSubsystem extends SubsystemBase {
    // double angle;
    public double x ;
    public double id ;
    double y ;
    public double area ;
    double delta ; 
  
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    public SwerveTrackingSubsystem(){
    }
    @Override
    public void periodic() {
        x = tx.getDouble(0.0);
        id = tid.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        SmartDashboard.putNumber("distance", getDistance());
        SmartDashboard.putNumber("limelightx", x);
        SmartDashboard.putNumber("limelighty", y);
        SmartDashboard.putNumber("limelighta", area);
        SmartDashboard.putNumber("limelightid", id);
        if (getDistance() > 7){
            if (Constants.VisionConstants.SpeakerID == 4){
                Constants.VisionConstants.SpeakerID = 1;
            } else if (Constants.VisionConstants.SpeakerID == 7) {
                Constants.VisionConstants.SpeakerID = 2;
            }
        } else if (getDistance() < 7){
            if (Constants.VisionConstants.SpeakerID == 1){
                Constants.VisionConstants.SpeakerID = 4;
            } else  if (Constants.VisionConstants.SpeakerID == 2) {
                Constants.VisionConstants.SpeakerID = 7;
            }

        }
    }
    
    public Command AimAtSpeaker(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
        System.out.println(Constants.VisionConstants.SpeakerID);
        Command setPipelineCommand = this.run(
            () -> pipeline.setDouble(Constants.VisionConstants.SpeakerID)
            );
            setPipelineCommand.addRequirements(this);
            Command rotateSwerveCommand = new TeleopSwerve(
                s_Swerve,
                translationSup,
                strafeSup,
                () -> getRotation(0),
                robotCentricSup,
                () -> false,
                () -> false
            );
        return setPipelineCommand.alongWith(rotateSwerveCommand);
    }

    public void setPipelineSpeaker(){
        pipeline.setDouble(Constants.VisionConstants.SpeakerID);
    }
    
    public double getDistance(){
        double area = ta.getDouble(0.0);
        double oneSide = Math.sqrt(area);
        double distance = 4.83/oneSide; 
        return distance;
    }

    public double getTranslation(double targetDistance){
        double distance = getDistance();
        if (Double.isInfinite(distance)){
            return 0;
        }
        else{
            return targetDistance-distance;
        }
    }

    public double getRotation(double targetAngle){
        // System.out.println("id: " + id);
        //adjusting for mounting angle offset
        targetAngle += 2;
        if (id <= 0){
            System.out.println("id is 0");
            return 0;
        }
        else{
            if (tx.getDouble(0.0) > (targetAngle+10)){
                return (tx.getDouble(0.0)-targetAngle)*-0.03;
            } else  if (tx.getDouble(0.0) > (targetAngle+5)){
                return (tx.getDouble(0.0)-targetAngle)*-0.04;
            } else {
                return (tx.getDouble(0.0)-targetAngle)*-0.05;
            }
        }
    }

        public double getRotationAuton(double targetAngle){
        // System.out.println("id: " + id);
        //adjusting for mounting angle offset
        targetAngle += 2;
        if (id <= 0){
            System.out.println("id is 0");
            return 0;
        }
        else{
            delta = tx.getDouble(0.0) - targetAngle;
            if (Math.abs(delta) > 10){
                return delta*-0.02;
            } else  if (Math.abs(delta) > 5){
                return delta*-0.035;
            } else {
                return delta*-0.04;
            }
        }
    }

      // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-front") * kP;
    targetingForwardSpeed *= Constants.Swerve.maxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }


   
}
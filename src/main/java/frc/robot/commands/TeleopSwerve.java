package frc.robot.commands;

import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveTrackingSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {
    private SwerveTrackingSubsystem traking;    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier leftStrafeSup;
    private BooleanSupplier rightStrafeSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier leftStrafeSup, BooleanSupplier rightStrafeSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.leftStrafeSup = leftStrafeSup;
        this.rightStrafeSup = rightStrafeSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(leftStrafeSup.getAsBoolean()) {
            final var rot_limelight = traking.limelight_aim_proportional();
            rotationVal = rot_limelight;

            final var forward_limelight = traking.limelight_range_proportional();
            translationVal = forward_limelight;

            //while using Limelight, turn off field-relative driving.
            robotCentricSup = () -> false;

            strafeVal = ((100 - traking.area) / 10) *0.04;
        } else if (rightStrafeSup.getAsBoolean()){
            final var rot_limelight = traking.limelight_aim_proportional();
            rotationVal = rot_limelight;

            final var forward_limelight = traking.limelight_range_proportional();
            translationVal = forward_limelight;

            //while using Limelight, turn off field-relative driving.
            robotCentricSup = () -> false;

            strafeVal = -(((100 - traking.area) / 10) *0.04);
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
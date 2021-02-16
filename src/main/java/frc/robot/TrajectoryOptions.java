package frc.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.subsystems.*;

public class TrajectoryOptions implements Sendable {

    public Drivetrain drivetrain;
    public BallSuck intake;
    public PathPoint[] path;
    public double initialRotation;
    public boolean useGyroRotation;

    public TrajectoryOptions(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.useGyroRotation = false;
        this.initialRotation = 0;
    }

    public TrajectoryOptions addIntake(BallSuck intake){
        this.intake = intake;
        return this;
    }

    public TrajectoryOptions addPath(PathPoint[] path){
        this.path = path;
        return this;
    }

    public TrajectoryOptions addInitialRotation(double initialRotation){
        this.initialRotation = initialRotation;
        return this;
    }

    public TrajectoryOptions useGyro(){
        this.useGyroRotation =true;
        return this;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
    }
    
}
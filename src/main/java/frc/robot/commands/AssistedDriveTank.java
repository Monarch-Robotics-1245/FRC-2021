package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.MotorControlPID;
import frc.robot.OI;
import frc.robot.PathPoint;
import frc.robot.Robot;
import frc.robot.TrajectoryOptions;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID;

public class AssistedDriveTank extends CommandBase {

    Drivetrain drivetrain;
    BallSuck ballsuck;
    PathPoint[] positions;

    int index;
    public boolean finished;

    private MotorControlPID leftPID, rightPID;

    //max distance (in meters) we can be in order for it to be valid.
    final double maxErrorDistance = 1.0;

    //max rotation we can be in radians
    final double maxErrorRotation = 0.5;

    //meters per second.
    final double targetVelocity = 1.3;
    
    public double initialRotation;
    public boolean useGyroRotation;
    
    private NetworkTable nt;

    public AssistedDriveTank(TrajectoryOptions options){
        this.drivetrain = options.drivetrain;
        this.initialRotation = options.initialRotation;
        this.useGyroRotation = options.useGyroRotation;
        if(options.intake !=null){
            this.ballsuck = options.intake;
            addRequirements(drivetrain,ballsuck);
        }
        else{
            addRequirements(drivetrain);
        }
        if(options.path !=null){
            this.positions = options.path;
        }
        else{
            PathPoint[] path = {
                new PathPoint(0,0),
                new PathPoint(-1,0),
            };
            this.positions = path;
        }
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt = inst.getTable("PathFollowing");
    }

    @Override
    public void initialize() {
        index = 1;
        if(useGyroRotation){
            drivetrain.resetOdometry(new Pose2d(positions[0].x, positions[0].y, new Rotation2d(initialRotation * Math.PI / 180.0 + drivetrain.getGyro().getRotation2d().getRadians())));
        }
        else{
            drivetrain.resetOdometry(new Pose2d(positions[0].x, positions[0].y, new Rotation2d(initialRotation * Math.PI / 180.0)));
        }
        finished = false;
        leftPID = new MotorControlPID(targetVelocity,1.0,1.0,0.03,0.02);
        rightPID = new MotorControlPID(targetVelocity,1.0,1.0,0.03,0.02);
    }

    public void updatePath(PathPoint[] newPath){
        System.out.println("UPDATING PATH");
        positions = newPath;
        index = 1;
        if(useGyroRotation){
            drivetrain.resetOdometry(new Pose2d(positions[0].x, positions[0].y, drivetrain.getGyro().getRotation2d()));
        }
        else{
            drivetrain.resetOdometry(new Pose2d(positions[0].x, positions[0].y, new Rotation2d(initialRotation * Math.PI / 180.0)));
        }
        finished = false;
        // leftPID = new MotorControlPID(targetVelocity,1.0,1.0,0.15,0.06);
        // rightPID = new MotorControlPID(targetVelocity,1.0,1.0,0.15,0.06);
    }

    public void updatePath(PathPoint[] newPath, double rotation){
        this.initialRotation = rotation;
        updatePath(newPath);
    }

    @Override
    public void execute() {
        //Get the current position the robot is trying to get to.
        if(index>=positions.length){
            drivetrain.tankDrive(0, 0);
            return;
        }
        PathPoint target = positions[index];
        //Get the error in the X (index 0), Y (index 1), total distance (index 3), and rotation (index 4)
        double[] errors = errorFromPoint(target);
        //Get the values of each encoder
        double[] encoders = { drivetrain.getLeftEncoder().getRate(), drivetrain.getRightEncoder().getRate() };

        //Calulate how fast the "inside" wheel should spin based on how far we should
        double encoderScalar = 1 - Math.abs(errors[3]);
        // if(Math.abs(errors[3])>=3*Math.PI/4 && errors[2] > 0.5){
        //  encoderScalar = 1 - Math.PI + Math.abs(errors[3]);
        // } 
        if(encoderScalar<-1.0){
            encoderScalar = -1.0;
        }
        //Set the "inside" wheel to spin at a slower rate (from above)
        if((!target.backwards && errors[3]<=0) || (target.backwards && errors[3]>=0)){
            rightPID.setTarget(targetVelocity * encoderScalar * backwards(target) * target.velocityScalar);
            leftPID.setTarget(targetVelocity * backwards(target) * target.velocityScalar);
        }
        else{
            leftPID.setTarget(targetVelocity * encoderScalar * backwards(target) * target.velocityScalar);
            rightPID.setTarget(targetVelocity * backwards(target) * target.velocityScalar);
        }

        // if(Math.abs(errors[3])>=2*Math.PI/3){
        //     rightPID.setTarget(-rightPID.getTarget());
        //     leftPID.setTarget(-leftPID.getTarget());
        // }
        
        //Get the power to apply to each motor based on how fast encoders are spinning.
        double leftSpeed = leftPID.getSpeed(encoders[0]);
        double rightSpeed = rightPID.getSpeed(encoders[1]);

        // nt.getEntry("leftTarget").setDouble(leftPID.getTarget());
        // nt.getEntry("rightTarget").setDouble(rightPID.getTarget());
        // nt.getEntry("leftEncoder").setDouble(encoders[0]);
        // nt.getEntry("rightEncoder").setDouble(encoders[1]);
        nt.getEntry("index").setNumber(index);
        // nt.getEntry("errorX").setDouble(errors[0]);
        // nt.getEntry("errorY").setDouble(errors[1]);
        // nt.getEntry("errorDist").setDouble(errors[2]);
        // nt.getEntry("errorAngle").setDouble(errors[3]);
        nt.getEntry("Left Motor Speed").setDouble(leftSpeed);
        nt.getEntry("Right Motor Speed").setDouble(rightSpeed);
        // nt.getEntry("velocityScalar").setDouble(target.velocityScalar);
        // nt.getEntry("useIntake").setBoolean(target.intake);


        if(!OI.rightJoystick.getTrigger() && !OI.leftJoystick.getTrigger()){
            if(Robot.isSimulation()){
                drivetrain.tankDrive(
                    -OI.deadZone(OI.xboxController.getY(GenericHID.Hand.kLeft), Constants.getDeadZone()), 
                    -OI.deadZone(OI.xboxController.getY(GenericHID.Hand.kRight), Constants.getDeadZone())
                    );
            }
            else{
                drivetrain.tankDrive(
                    -OI.deadZone(OI.leftJoystick.getY(), Constants.getDeadZone()), 
                    -OI.deadZone(OI.rightJoystick.getY(), Constants.getDeadZone())
                    );
            }
        }

        // drivetrain.tankDrive(leftSpeed, rightSpeed);
        if(ballsuck !=null){
            if(target.intake){
                ballsuck.turnOnIntake();
                ballsuck.turnOnHandle();
            }
            else{
                ballsuck.turnOffIntake();
                ballsuck.turnOffHandle();
            }
        }

        //If we are close to the target point, advance to the next index. If it is the last point, finish the command.
        if(errors[2]<maxErrorDistance){
            index++;
            leftPID.reset();
            rightPID.reset();
            if(index>=positions.length){
                finished = true;
                drivetrain.tankDriveVolts(0, 0);
            }
        }
    }

    double[] errorFromPoint(PathPoint point){
        //calulate the distance we are away in each direction.
        double errorX = (point.x - drivetrain.getPathPose().getX()) * backwards(point);
        double errorY = (point.y - drivetrain.getPathPose().getY()) * backwards(point);
        //calculate the total distance using the pythagorean theorem.
        double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);
        double robotAngle = drivetrain.getPathPose().getRotation().getRadians();
        double robotX = Math.cos(robotAngle);
        double robotY = Math.sin(robotAngle);
        if(distanceError==0){
            distanceError = 1;
        }
        double normalErrorX = errorX/distanceError;
        double normalErrorY = errorY/distanceError;

        //calulcat the angle of the robot using a Vector dot product
        double dot = robotX * normalErrorX + robotY * normalErrorY;
        double angleError = Math.acos(dot);
        //figure out if we need to flip the angle based on which direction we need to spin
        //https://stackoverflow.com/a/13221874
        double dotRotated = -robotY * normalErrorX + robotX * normalErrorY;
        if(dotRotated<0){
            angleError *= -1;
        }

        //return the four errors as an array
        double[] error = {errorX, errorY, distanceError, angleError};
        return error;
    }

    public int backwards(PathPoint p){
        return p.backwards ? -1 : 1;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
        System.out.println("DONE WITH ALIGN");
        if(ballsuck !=null){
            ballsuck.turnOffIntake();
            ballsuck.turnOffHandle();
        }
    }
}

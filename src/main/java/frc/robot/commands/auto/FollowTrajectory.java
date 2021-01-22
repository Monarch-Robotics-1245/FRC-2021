package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends RamseteCommand  {

    Drivetrain drivetrain;
    Trajectory trajectory;

    public FollowTrajectory(Trajectory trajectory, Drivetrain trajectoryDrive) {
        super(
            trajectory,
            trajectoryDrive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            trajectoryDrive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            trajectoryDrive::tankDriveVolts,
            trajectoryDrive
        );
        drivetrain = trajectoryDrive;
        this.trajectory = trajectory;
    }

    @Override 
    public void initialize(){
        super.initialize();
        drivetrain.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        super.execute();
        System.out.println(drivetrain.getPose());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        System.out.println("FINISHED FOLLOWING TRAJECTORY");
        drivetrain.tankDriveVolts(0.0, 0.0);
    }
}

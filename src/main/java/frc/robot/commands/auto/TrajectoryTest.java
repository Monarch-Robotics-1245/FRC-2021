package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.OldDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TrajectoryTest extends SequentialCommandGroup {
    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public TrajectoryTest(Turret turret, Drivetrain drivetrain, BallSuck ballsuck){
        
            // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10.0);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);


        //First test to see if it can follow a basic path
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.1518347624, 0.0532079543, new Rotation2d(7.799128318 * 3.1415926535/180.0)),
            List.of(
                new Translation2d(0,3),
                new Translation2d(1,10)
            ),
            new Pose2d(2, 15, new Rotation2d(4.410397358 * 3.1415926535/180.0)),
            config
        );

        // addCommands(new FollowTrajectory(exampleTrajectory, drivetrain));


        //Using a simple path generated in Pathweaver:
        String testJson = "paths/Barell.wpilib.json";
        Trajectory testTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(testJson);
            testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testJson, ex.getStackTrace());
        }
        addCommands(new FollowTrajectory(testTrajectory, drivetrain));

        String barell2Json = "paths/Barell2.wpilib.json";
        Trajectory barell2Trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barell2Json);
            barell2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testJson, ex.getStackTrace());
        }
        // addCommands(new FollowTrajectory(barell2Trajectory, drivetrain));
        
        String barell3Json = "paths/Barell3.wpilib.json";
        Trajectory barell3Trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barell3Json);
            barell3Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testJson, ex.getStackTrace());
        }
        // addCommands(new FollowTrajectory(barell3Trajectory, drivetrain));
        
        //The barell path generated in Pathweaver:
        /* String barellJson = "paths/Test.wpilib.json";
        Trajectory barellTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barellJson);
            barellTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + barellJson, ex.getStackTrace());
        }
        drivetrain.resetOdometry(barellTrajectory.getInitialPose());
        addCommands(new FollowTrajectory(barellTrajectory, drivetrain)); */
    }
}
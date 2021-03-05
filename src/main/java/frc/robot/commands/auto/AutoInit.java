/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.Timer;


/**
 * Gets everything that needs to outside of the frame perimiter
 */
public class AutoInit extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BallSuck suck;
    private final Drivetrain drivetrain;

    private Timer timer;

    /**
     * @param subsystem The BallSuck subsystem {@link BallSuck} so that we can extend the ballSuck outside of frame perimeter
     */
    public AutoInit(BallSuck suck, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.suck = suck;
        timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(suck);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.getGyro().reset();
        drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0, 0.0)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        suck.intakeBackwards();
    }

    // Called once the command ends or is interrupted, sets motors to stop moving
    @Override
    public void end(boolean interrupted) {
        suck.turnOffIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //returns false while the system hasn't activated fully yet
        return timer.get()>0.2;
    }
}
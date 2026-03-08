package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NextFTCPatch.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous
public class BlueCloseNineBallAuto extends NextFTCOpMode{
    public Path p1;
    public Path p2;
    public Path p3;
    public Path p4;
    public Path p5;

    public static double hoodPosition = 0;
    public static double velocity = 0;
    public BlueCloseNineBallAuto(){
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE, Intake.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    public Command setToPositionOne(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_ONE));
    }

    public Command setToPositionTwo(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_TWO));
    }

    public Command setToPositionThree(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_THREE));
    }

    public Command transferFlick(){
        return new SequentialGroupFixed(
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown());

    }
    public Command shootThree(){
        return new SequentialGroupFixed(
                Intake.INSTANCE.idle(),
                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)),
                setToPositionOne(),
                new Delay(0.7),
                Turret.INSTANCE.waitToShoot(),
                Transfer.INSTANCE.transferDown(),
                new Delay(0.5),
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new Delay(0.7),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                setToPositionTwo(),
                new Delay(0.7),
                Turret.INSTANCE.waitToShoot(),
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new Delay(0.7),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                setToPositionThree(),
                new Delay(0.7),
                Turret.INSTANCE.waitToShoot(),
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY))
                );
    }
    public Command intakeMode(){
        return new SequentialGroupFixed(
        new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE))
        );
    }


    public void buildPaths(){
        p1 = new Path(new BezierLine(
                new Pose(27.184, 130.041),
                new Pose(61.531, 94.22434693877551)
        ));
        p1.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        p2 = new Path(new BezierCurve(
                new Pose(61.531, 94.22434693877551),
                new Pose(63.55102040816326, 54.73469387755101),
                new Pose(36.20408163265305, 59.71432653061225)
        ));
        p2.setConstantHeadingInterpolation(Math.toRadians(180));

        p3 = new Path(new BezierLine(
                new Pose(36.20408163265305, 59.71432653061225),
                new Pose(21.632653061224488, 59.755102040816325)
        ));
        p3.setConstantHeadingInterpolation(Math.toRadians(180));

        p4 = new Path(new BezierLine(
                new Pose(21.632653061224488, 59.755102040816325),
                new Pose(61.83673469387754, 93.87755102040816)
        ));
        p4.setConstantHeadingInterpolation(Math.toRadians(180));

        p5 = new Path(new BezierLine(
                new Pose(22.06122448979592, 84.143),
                new Pose(61.612, 85.122)
        ));
        p5.setConstantHeadingInterpolation(Math.toRadians(180));
    }

    public Command autonomousRoutine(){
        return new SequentialGroupFixed(
                new FollowPath(p1),
                shootThree(),
                intakeMode(),
                new FollowPath(p2),
                new FollowPath(p3,true,0.75),
                new FollowPath(p4),
                shootThree(),
                shootThree()
//                new FollowPath(p3),
//                new FollowPath(p4),
//                new FollowPath(p5)
        );
    }
    @Override
    public void onStartButtonPressed(){
        buildPaths();
        follower().setStartingPose(new Pose(27.184, 130.041, Math.toRadians(142)));
//        Turret.INSTANCE.setToZero().schedule();
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 27.184, 130.041, AngleUnit.DEGREES, 142));
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());

        Turret.INSTANCE.status(telemetry);
        Spindexer.INSTANCE.status(telemetry);
        telemetry.update();

        if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            Intake.INSTANCE.on().schedule();
            if(Spindexer.INSTANCE.freePosition()!=-1){
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
            }
        }

        else if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT){
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()).schedule();
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE,10).schedule();
            velocity = Turret.INSTANCE.distanceToVelocity(follower().getPose().getX(), follower().getPose().getY(), Aliance.BLUE);
            hoodPosition = Turret.INSTANCE.distanceToPosition(follower().getPose().getX(), follower().getPose().getY(), Aliance.BLUE);
        }

        Turret.INSTANCE.setVelocity(velocity).schedule();
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

    }
}


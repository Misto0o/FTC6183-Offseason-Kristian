package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous
public class BlueCloseTwelveBallAuto extends NextFTCOpMode {
//    Robot robot = new Robot(Aliance.BLUE);

    public static double hoodPosition = 0;
    public static double velocity = 0;

    public BlueCloseTwelveBallAuto(){
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
    private Path scorePreload;
    private Path pickUpFirstRow;
    private Path returnOne;
    private Path pickUpSecondRow;
    private Path gateOne;
    private Path gateTwo;
    private Path returnTwo;
    private Path pickUpThirdRow;
    private Path end;
    public boolean shootcycle = false;

    //private Pose startPose = new Pose();
//private Pose firstRowPose = new Pose();
//private Pose returnPose = new Pose();
//private Pose secondRowPose = new Pose();
//private Pose thirdRowPose = new Pose();
//private Pose endPose = new Pose();
    public Command shootThree(){
        return new SequentialGroupFixed(
            new InstantCommand(() -> shootcycle = true),
            new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)),
            Intake.INSTANCE.idle(),
            new Delay(0.5),
            setToPositionOne(),
            new Delay(0.5),
            Transfer.INSTANCE.transferUp(),
            new Delay(0.5),
            Transfer.INSTANCE.transferDown(),
            new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
            setToPositionTwo(),
            new Delay(0.5),
            Transfer.INSTANCE.transferUp(),
            new Delay(0.5),
            Transfer.INSTANCE.transferDown(),
            new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
            setToPositionThree(),
            new Delay(0.5),
            Transfer.INSTANCE.transferUp(),
            new Delay(0.5),
            Transfer.INSTANCE.transferDown(),
            new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
            new Delay(0.2),
            new InstantCommand(()->shootcycle = false)
        );
    }
    public Command intakeMode(){
        return new SequentialGroupFixed(
                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE)),
                Intake.INSTANCE.on()
                );
    }
    public void buildPaths(){
        pickUpFirstRow = new Path(
                        new BezierCurve(
                                new Pose(24.980, 127.469),
                                new Pose(73.112, 78.684),
                                new Pose(16.714, 83.755)
                        )
                );

        pickUpFirstRow.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        returnOne = new Path(new BezierCurve(
                new Pose(16.714, 83.755),
                new Pose(62.061, 83.816)
        ));
        returnOne.setConstantHeadingInterpolation(Math.toRadians(180));

        pickUpSecondRow = new Path(new BezierCurve(
                new Pose(62.061, 83.816),
                new Pose(61.306, 57.449),
                new Pose(16.061, 59.735)
        )
        );
        pickUpSecondRow.setTangentHeadingInterpolation();

        gateOne = new Path(new BezierLine(
                new Pose(16.061, 59.735),

                new Pose(16.224, 71.449)
        )
        );

        gateOne.setTangentHeadingInterpolation();
        gateTwo = new Path( new BezierLine(
                new Pose(16.224, 71.449),

                new Pose(15.347, 70.959)
        ));
        gateTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        returnTwo = new Path( new BezierLine(
                new Pose(15.347, 70.959),

                new Pose(73.020, 71.510)
        )
        );
        returnTwo.setTangentHeadingInterpolation();


        pickUpThirdRow = new Path(new BezierCurve(
                new Pose(73.020, 71.510),
                new Pose(75.306, 35.633),
                new Pose(13.367, 34.939)
        )
        );

        pickUpThirdRow.setTangentHeadingInterpolation();

        end = new Path( new BezierLine(
                new Pose(13.367, 34.939),

                new Pose(72.918, 71.449)
        )
        );
        end.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142));
    }

    public Command autonomousRoutine(){
        return new SequentialGroupFixed(
                Turret.INSTANCE.waitToShoot(),
                shootThree(),
                new FollowPath(pickUpFirstRow),
                intakeMode(),
                new Delay(0.1),
                new FollowPath(returnOne),
                new Delay(0.1),
                Turret.INSTANCE.waitToShoot(),
                shootThree()
//                new FollowPath(pickUpSecondRow),
//                new Delay(0.1),
//                new FollowPath(gateOne),
//                new Delay(0.1),
//                new FollowPath(gateTwo),
//                new Delay(0.1),
//                new FollowPath(returnTwo),
//                shootThree(),
//                new Delay(0.1),
//                new FollowPath(pickUpThirdRow),
//                new Delay(0.1),
//                shootThree(),
//                new FollowPath(end)
        );
    }

    @Override
    public void onStartButtonPressed(){
        buildPaths();
        follower().setStartingPose(new Pose(24.980, 127.469, Math.toRadians(142)));
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());
        telemetry.update();
        if(Spindexer.INSTANCE.freePosition()!=-1 && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE && !shootcycle) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE,10).schedule();
        }
//        else if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT && Spindexer.INSTANCE.filledPosition()!=-1 && !shootcycle ){
//            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.filledPosition()]).schedule();
//            velocity = Turret.INSTANCE.distanceToVelocity(Pinpoint.INSTANCE.getPosX(),Pinpoint.INSTANCE.getPosY());
//        }
        else{
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()).schedule();
            velocity = Turret.INSTANCE.distanceToVelocity(follower().getPose().getX() , follower().getPose().getY(), Aliance.BLUE);
            hoodPosition = Turret.INSTANCE.distanceToPosition(follower().getPose().getX() , follower().getPose().getY(), Aliance.BLUE);

        }
        Turret.INSTANCE.setVelocity(velocity).schedule();
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

    }
}


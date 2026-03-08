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
public class BlueCloseTwelveBallAutoP2 extends NextFTCOpMode{

    private Path scorePreload;
    private Path pickUpFirstRow;
    private Path returnOne;
    private Path pickUpSecondRow;
    private Path gateOne;
    private Path gateTwo;
    private Path returnTwo;
    private Path pickUpThirdRow;
    private Path end;

    public BlueCloseTwelveBallAutoP2(){
        addComponents(
                //new SubsystemComponent(Spindexer.INSTANCE, Intake.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(
                new Pose(24.980, 127.469),
                new Pose(61.531, 84.857)
        ));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        pickUpSecondRow = new Path(new BezierCurve(
                new Pose(61.531, 84.857),
                new Pose(68.908, 56.918),
                new Pose(17.837, 59.898)
        ));
        pickUpSecondRow.setConstantHeadingInterpolation(Math.toRadians(180));

        gateOne = new Path(new BezierLine(
                new Pose(17.837, 59.898),
                new Pose(18.041, 67.816)
        ));
        gateOne.setConstantHeadingInterpolation(Math.toRadians(90));

        gateTwo = new Path(new BezierLine(
                new Pose(18.041, 67.816),
                new Pose(13.755, 67.714)
        ));
        gateTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        returnTwo = new Path(new BezierLine(
                new Pose(13.755, 67.714),
                new Pose(61.531, 72.449)
        ));
        returnTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        Path path6 = new Path(new BezierLine(
                new Pose(61.531, 72.449),
                new Pose(61.449, 84.918)
        ));
        path6.setConstantHeadingInterpolation(Math.toRadians(90));

        pickUpFirstRow = new Path(new BezierLine(
                new Pose(61.449, 84.918),
                new Pose(17.837, 84.143)
        ));
        pickUpFirstRow.setConstantHeadingInterpolation(Math.toRadians(180));

        returnOne = new Path(new BezierLine(
                new Pose(17.837, 84.143),
                new Pose(61.612, 85.122)
        ));
        returnOne.setConstantHeadingInterpolation(Math.toRadians(180));

        pickUpThirdRow = new Path(new BezierCurve(
                new Pose(61.612, 85.122),
                new Pose(58.592, 30.673),
                new Pose(15.776, 35.633)
        ));
        pickUpThirdRow.setTangentHeadingInterpolation();

        end = new Path(new BezierLine(
                new Pose(15.776, 35.633),
                new Pose(61.490, 85.061)
        ));
        end.setTangentHeadingInterpolation();
    }

    public Command autonomousRoutine(){
        return new SequentialGroupFixed(
               new FollowPath(scorePreload),
                new FollowPath(pickUpSecondRow)
        );
    }
    @Override
    public void onStartButtonPressed(){
        buildPaths();
        follower().setStartingPose(new Pose(24.980, 127.469, Math.toRadians(142)));
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());
        telemetry.update();
    }
}

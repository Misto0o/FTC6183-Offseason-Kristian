package org.firstinspires.ftc.teamcode.Auto.PairedAuto.bttdb;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTCPatch.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class BlueFarAuto extends NextFTCOpMode{
    Robot robot = new Robot(Aliance.BLUE);
    public BlueFarAuto(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(robot, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private Path scorePreload;
    private Path pickUpLastRow;
    private Path returnOne;
    private Path returnTwo;
    private Path pickUpLoadingZoneOne;
    private Path pickUpLoadingZoneTwo;


    private Pose startPose = new Pose(56,8);
    private Pose shootPose = new Pose(72,72);
    private Pose lastrowPose = new Pose(14.122, 35.816);
    private Pose loadingZonePose = new Pose(5,8);
    private Pose loadingZonePose2 =  new Pose(5.551, 20.265);
    private Pose endPose =  new Pose(72.020, 23.327);



    public void buildPaths(){
//        scorePreload = new Path(new BezierLine(startPose,shootPose));

        pickUpLastRow = new Path(new BezierCurve(
                startPose,
                new Pose(51.245, 40.959),
                lastrowPose
        ));

        pickUpLastRow.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        returnOne = new Path(new BezierLine(
                shootPose,
                startPose
        )
        );

        returnOne.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(90));

        pickUpLoadingZoneOne = new Path(new BezierLine(
                startPose,
                loadingZonePose
        )
        );

        pickUpLoadingZoneOne.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        returnTwo = new Path(new BezierLine(
                loadingZonePose,
                startPose
        )
        );
        returnTwo.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(90));
        pickUpLoadingZoneTwo = new Path(
                        new BezierCurve(
                                startPose,
                                new Pose(31.276, 20.316),
                                loadingZonePose2
                        )
                );

        pickUpLoadingZoneTwo.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        returnTwo = new Path(new BezierLine(
                loadingZonePose2,
                endPose)
        );

        returnTwo.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(90));
    }

    public Command buildAuto(){
        return new SequentialGroupFixed(
                new FollowPath(pickUpLastRow),
                new FollowPath(returnOne),
                new FollowPath(pickUpLoadingZoneOne),
                new FollowPath(returnTwo)
        );
    }

    @Override
    public void onStartButtonPressed(){
        buildPaths();
        follower().setStartingPose(new Pose(56,8,Math.toRadians(90)));
        buildAuto().schedule();

    }

    @Override
    public void onUpdate(){
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());
        telemetry.update();
    }
}

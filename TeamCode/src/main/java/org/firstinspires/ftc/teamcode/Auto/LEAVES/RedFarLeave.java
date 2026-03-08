package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import org.firstinspires.ftc.teamcode.Pedro.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class RedFarLeave extends NextFTCOpMode {
    public RedFarLeave(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private Path farLeave = new Path();
    public void buildPath(){
        farLeave = new Path(new BezierCurve(
                new Pose(85.224,9),
                new Pose(85.38775510204086, 60)));
        farLeave.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public Command autoMoveForward(){
        return new FollowPath(farLeave);
    }

    @Override
    public void onStartButtonPressed(){
        buildPath();
        follower().setStartingPose(new Pose(85.38775510204086, 9, Math.toRadians(90)));
        autoMoveForward().schedule();
    }
    @Override
    public void onUpdate(){
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());
        telemetry.update();
    }
}

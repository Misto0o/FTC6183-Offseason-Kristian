package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class BlueCloseLeave extends NextFTCOpMode {
    public BlueCloseLeave(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
//                new PedroComponent(Constants::createFollower)
        );
    }
    private Path closeLeave = new Path();

    public void buildPath(){
        closeLeave = new Path(new BezierCurve(
                new Pose(26.999999999999996,129.48979591836738),
                new Pose(54.16326530612246, 98.75510204081631)));
        closeLeave.setLinearHeadingInterpolation(Math.toRadians(142),Math.toRadians(90));
    }
    public Command autoMoveForward(){
        return new FollowPath(closeLeave);
    }

    @Override
    public void onStartButtonPressed(){
        buildPath();
        follower().setStartingPose(new Pose(72,72,Math.toRadians(90)));
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

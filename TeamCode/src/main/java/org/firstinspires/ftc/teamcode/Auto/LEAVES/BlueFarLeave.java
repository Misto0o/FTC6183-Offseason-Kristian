package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Pedro.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class BlueFarLeave extends NextFTCOpMode {
    public BlueFarLeave(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Pinpoint.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    private Path farLeave = new Path();
    public void buildPath(){
        farLeave = new Path(new BezierCurve(
                new Pose(58.95918367346939,9),
                new Pose(58.75510204081634, 20)));
        farLeave.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public Command autoMoveForward(){
        return new FollowPath(farLeave);
    }

    @Override
    public void onStartButtonPressed(){
        buildPath();
        follower().setStartingPose(new Pose(58.75510204081634, 9, Math.toRadians(90)));
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 58.75510204081634, 9, AngleUnit.DEGREES, 90));
        autoMoveForward().schedule();
    }

    @Override
    public void onUpdate(){
        Pinpoint.INSTANCE.periodic();
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());
        telemetry.update();
    }
}


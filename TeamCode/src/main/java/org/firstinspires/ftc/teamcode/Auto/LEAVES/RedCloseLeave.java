package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class RedCloseLeave extends NextFTCOpMode {
    public RedCloseLeave(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Pinpoint.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    private Path closeLeave = new Path();
    public void buildPath(){
        closeLeave = new Path(new BezierCurve(
                new Pose(116.26530612244898,130.95918367346943),
                new Pose(90.89795918367348, 103.53061224489794)));
        closeLeave.setLinearHeadingInterpolation(Math.toRadians(38),Math.toRadians(90));
    }
    public Command autoMoveForward(){
        return new FollowPath(closeLeave);
    }

    @Override
    public void onStartButtonPressed(){
        buildPath();
        follower().setStartingPose(new Pose(116.26530612244898,130.95918367346943,Math.toRadians(38)));
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 116.26530612244898,130.95918367346943, AngleUnit.DEGREES, 38));
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

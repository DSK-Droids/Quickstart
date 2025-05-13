package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;







import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robocol.Command;
import com.pedropathing.commands.FollowPath; //Not importing

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "5+1 Specimen Auto With Sample Drop ")
public class Test extends LinearOpMode {



    private Follower follower;

    /**
     * Starting Position of our robot
     */


    private Path scorePreload;


    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
    }

    public void runOpMode(){

        buildPaths();


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        CommandScheduler.






        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            follower.update();


        }

    }
}
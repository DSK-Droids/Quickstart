package pedroPathing.navInfo;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

private Path scorePreload, park;
private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

public class GeneratedPaths {
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose scorePose = new Pose(18, 126, Math.toRadians(-45));

    private final Pose collect1Pose = new Pose(26, 121, Math.toRadians(0));

    private final Pose collect2Pose = new Pose(26, 131, Math.toRadians(0));

    private final Pose collect3Pose = new Pose(26, 141, Math.toRadians(0));
    public static PathBuilder builder = new PathBuilder();
    public void buildPaths(){
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scorePose)));
    }
    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(startPose), new Point(scorePose)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierLine(
                            new Point(18.000, 126.000, Point.CARTESIAN),
                            new Point(26.000, 121.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(26.000, 121.000, Point.CARTESIAN),
                            new Point(18.000, 126.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
            .setReversed(true)
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierLine(
                            new Point(18.000, 126.000, Point.CARTESIAN),
                            new Point(26.000, 131.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierLine(
                            new Point(26.000, 131.000, Point.CARTESIAN),
                            new Point(18.000, 126.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
            .setReversed(true)
            .build();
}

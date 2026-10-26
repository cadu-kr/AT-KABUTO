package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.api.Paths;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;


@Autonomous(name = "TT-Auto")
public class TTauto extends OpMode {

    Follower follower;
    //DcMotor intake, lancador;
    Limelight3A limelight;
    double distanciaTag = -1;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose andaGanhaPonto = poseFactory.of(56, 12, 90);
    private final Pose intakeGardenStart = poseFactory.of(56, 12, 90);
    private final Pose intakeGarden = poseFactory.of(10, 8, 180);
    private final Pose lancar1Start = poseFactory.of(10, 8, 180);
    private final Pose lancar1 = poseFactory.of(63, 131, -96);
    private final Pose lancar1Control1 = poseFactory.of(31.3778, 131.9443, 0);
    private final Pose intakeFlowerStart = poseFactory.of(63, 131, -96);
    private final Pose intakeFlower = poseFactory.of(55, 130, 135);
    private final Pose intakeFlowerControl1 = poseFactory.of(59.8199, 121.5516, 0);
    private final Pose lanca2Start = poseFactory.of(55, 130, 135);
    private final Pose lanca2 = poseFactory.of(63, 131, -96);
    private final Pose estacionarStart = poseFactory.of(63, 131, -96);
    private final Pose estacionar = poseFactory.of(10, 116, 0);


    public Path andaGanhaPonto() {
        return Paths.line(start, andaGanhaPonto).linear(start, andaGanhaPonto);
    }

    public Path intakeGarden() {
        return Paths.line(intakeGardenStart, intakeGarden).linear(intakeGardenStart, intakeGarden);
    }

    public Path lancar1() {
        return Paths.curve(lancar1Start, lancar1Control1, lancar1).linear(lancar1Start, lancar1);
    }

    public Path intakeFlower() {
        return Paths.curve(intakeFlowerStart, intakeFlowerControl1, intakeFlower).linear(intakeFlowerStart, intakeFlower);
    }

    public Path lanca2() {
        return Paths.line(lanca2Start, lanca2).linear(lanca2Start, lanca2);
    }

    public Path estacionar() {
        return Paths.line(estacionarStart, estacionar).linear(estacionarStart, estacionar);
    }

    private Command initLimelight() {
        return instant(() -> {
            LLResult result = limelight.getLatestResult();

            if (result != null &&  result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (!tags.isEmpty()) {
                    Pose3D p = tags.get(0).getTargetPoseCameraSpace();
                    double x = p.getPosition().x;
                    double y = p.getPosition().y;
                    double z = p.getPosition().z;
                    distanciaTag = Math.sqrt(x*x + y*y + z*z);
                }
            }
        });
    }
    /*private Command ligaIntake() {
        return instant(() -> intake.setPower(1));
    }
    private Command desligaIntake() {
        return instant(() -> intake.setPower(0));
    }

    private Command ligaLancador() {
        return instant(() -> lancador.setPower(1));
    }
    private Command desligaLancador() {
        return  instant(() -> lancador.setPower(0));
    }*/

    private Command autoRoutine() {
        return sequential(
                follow(follower, andaGanhaPonto()),
                //ligaLancador(),
                waitMs(2000),
                //desligaLancador(),
                follow(follower, intakeGarden()),
                //ligaIntake(),
                waitMs(1000),
                //desligaIntake(),
                follow(follower, lancar1()),
                follow(follower, intakeFlower()),
                follow(follower, lanca2()),
                follow(follower, estacionar())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //lancador = hardwareMap.get(DcMotor.class, "lancador");
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}

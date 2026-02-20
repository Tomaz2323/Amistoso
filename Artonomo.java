package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Artonomo extends OpMode {

    public DcMotorEx flyWheel;
    public DcMotorEx flyWheel2;
    public DcMotorEx indexer;
    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = 1400;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // 1. Adicionados novos estados para controlar o tiro e a finalização
    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        WAIT_BEFORE_SHOOT,
        SPIN_UP,
        SHOOTING,
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(71.85321100917432, 8.792660550458724, Math.toRadians(90));
    private final Pose shootPose = new Pose(91.934, 100.470, Math.toRadians(45));

    private PathChain inicial;

    public void buildPaths(){
        inicial = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                //follower.followPath(inicial, true);
                setPathState(PathState.WAIT_BEFORE_SHOOT);
                break;

            case SHOOT_PRELOAD:
                // CORREÇÃO AQUI: Exigimos que o pathTimer seja maior que 0.5 segundos.
                // Isso garante que o follower já começou a se mover de fato e o isBusy() é confiável.
                if(pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy()){
                    setPathState(PathState.WAIT_BEFORE_SHOOT);
                }
                break;

            case WAIT_BEFORE_SHOOT:
                // Agora sim, ele só entra aqui com o robô totalmente parado na posição final!
                if(pathTimer.getElapsedTimeSeconds() > 2.0) {

                    // Passou o tempo de espera! Liga os motores
                    flyWheel.setVelocity(curTargetVelocity);
                    flyWheel2.setVelocity(curTargetVelocity);

                    // Vai para o estado de tiro
                    setPathState(PathState.SPIN_UP);
                }
                break;

            case SPIN_UP:
                // Espera MEIO SEGUNDO para as rodas atingirem a velocidade máxima
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {

                    // Levanta o Core Hex em 45 graus (36 ticks) para empurrar a bolinha
                    indexer.setTargetPosition(36);
                    indexer.setPower(0.6); // Força/Velocidade com que ele vai levantar (0.0 a 1.0)

                    // Vai para o estado de tiro
                    setPathState(PathState.SHOOTING);
                }
                break;

            case SHOOTING:
                // Espera 1.5 segundos (ajuste esse tempo conforme a necessidade do seu robô)
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    // Após o tempo de disparo, desliga os motores
                    flyWheel.setVelocity(0);
                    flyWheel2.setVelocity(0);

                    // Finaliza a rotina
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("Terminou o bagui aí - Tiro finalizado!");
                break;

            default:
                telemetry.addLine("Não to fazendo nada");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer(); // O timer reseta sozinho a cada troca de estado!
    }

    @Override
    public void init() {
        flyWheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flyWheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setTargetPosition(0);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setDirection(DcMotorSimple.Direction.FORWARD);

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Estado Atual", pathState);
        telemetry.addData("Shooter Vel", flyWheel.getVelocity());
        telemetry.update();
    }
}
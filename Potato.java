package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime; // <-- Importação do cronômetro adicionada

@TeleOp(name = "TeleOp Oficial", group = "TeleOp")
public class Potato extends OpMode {

    public DcMotorEx flyWheel;
    public DcMotorEx flyWheel2;
    public DcMotorEx indexer;

    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    private final double MAX_POWER = 1.0;

    // Variáveis para o controle de tempo do tiro
    private ElapsedTime timerDoShooter = new ElapsedTime();
    private boolean rotinaDeTiroAtiva = false;

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

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lr");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rr");

        // Configuração das direções exatas da sua MecanumConstants
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define o comportamento para frear quando soltar o controle
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TeleOp Inicializado com Sucesso!");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower * MAX_POWER);
        leftRear.setPower(backLeftPower * MAX_POWER);
        rightFront.setPower(frontRightPower * MAX_POWER);
        rightRear.setPower(backRightPower * MAX_POWER);

        if(gamepad1.bWasPressed()){
            flyWheel.setVelocity(1500);
            flyWheel2.setVelocity(1500);

            timerDoShooter.reset();
            rotinaDeTiroAtiva = true;
        }

        if (rotinaDeTiroAtiva) {
            // Após 1.5 segundos da ativação dos motores...
            if (timerDoShooter.seconds() > 1.5) {
                // Levanta o indexer em 45 graus (36 ticks)
                indexer.setTargetPosition(46);
                indexer.setPower(0.8);

                // Finaliza a rotina de espera
                rotinaDeTiroAtiva = false;
            }
        }

        // Botão para resetar o tiro (Desliga a roda e desce o indexer)
        if(gamepad1.aWasPressed()) {
            flyWheel.setVelocity(0);
            flyWheel2.setVelocity(0);

            indexer.setTargetPosition(0);
            indexer.setPower(0.4);

            rotinaDeTiroAtiva = false; // Cancela a rotina de tiro, caso esteja rodando
        }

        telemetry.addData("Força LF", frontLeftPower * MAX_POWER);
        telemetry.addData("Força RF", frontRightPower * MAX_POWER);
        telemetry.addData("Força LR", backLeftPower * MAX_POWER);
        telemetry.addData("Força RR", backRightPower * MAX_POWER);
        telemetry.addData("Rotina de Tiro", rotinaDeTiroAtiva ? "Aguardando 1.5s..." : "Pronto");
        telemetry.addData("Velocidade Shooter", flyWheel.getVelocity());
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/** Mira assistida para a celula pontuavel da colmeia BIOBUZZ. */
@TeleOp(name = "Marechal - Mira Colmeia", group = "Equipe")
public class MarechalMiraColmeia extends LinearOpMode {
    private static final double LANCADOR = 0.85;
    private static final double COLETOR = 0.70;
    private static final double PUX = 1.00;
    private static final double GARRA_CONTINUA = 0.25;
    private static final double GARRA_FECHADA = 0.45;
    private static final double GARRA_ABERTA = 0.55;

    // Giro lento e somente enquanto RB do controle 1 estiver pressionado.
    private static final double GANHO_MIRA = 0.015;
    private static final double GIRO_MINIMO = 0.10;
    private static final double GIRO_MAXIMO = 0.25;
    private static final double TOLERANCIA_GRAUS = 2.0;

    private DcMotor direito;
    private DcMotor esquerdo;
    private DcMotor lancador;
    private DcMotor coletor;
    private Servo garraPosicao;
    private CRServo garraContinua;
    private CRServo pux;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        direito = hardwareMap.get(DcMotor.class, "direito");
        esquerdo = hardwareMap.get(DcMotor.class, "esquerdo");
        lancador = hardwareMap.get(DcMotor.class, "lancador");
        coletor = hardwareMap.get(DcMotor.class, "coletor");
        garraPosicao = hardwareMap.tryGet(Servo.class, "garra");
        garraContinua = hardwareMap.tryGet(CRServo.class, "garra");
        pux = hardwareMap.tryGet(CRServo.class, "pux");

        direito.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lancador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        esquerdo.setPower(0);
        direito.setPower(0);
        lancador.setPower(0);
        coletor.setPower(0);
        if (pux != null) pux.setPower(0);
        if (garraContinua != null) garraContinua.setPower(0);

        try {
            List<WebcamName> cameras = hardwareMap.getAll(WebcamName.class);
            if (cameras.isEmpty()) {
                telemetry.addLine("Camera ausente da Robot Configuration.");
                telemetry.addLine("Adicione a webcam USB e ative a configuracao.");
                telemetry.update();
                return;
            }

            aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
            aprilTag.setDecimation(2);
            visionPortal = new VisionPortal.Builder()
                .setCamera(cameras.get(0))
                .addProcessor(aprilTag)
                .build();

            telemetry.addLine("Mira: X azul, B vermelho; segure RB para alinhar.");
            telemetry.addLine("D-pad esquerda/direita: ajuste fino de 1 grau.");
            telemetry.addLine("Y: alterna o lado da frente do robo.");
            telemetry.addLine("Controle 2: RT lancador, LT coleta, LB reverte, A/B garra.");
            telemetry.addLine("Na INIT: menu de 3 pontos > Camera Stream.");
            telemetry.update();

            waitForStart();
            String alianca = "";
            double ajusteGraus = 0.0;
            boolean esquerdaAntes = false;
            boolean direitaAntes = false;
            boolean cimaAntes = false;
            boolean baixoAntes = false;
            boolean yAntes = false;
            MarechalTracao tracao = new MarechalTracao();

            while (opModeIsActive()) {
                if (gamepad1.y && !yAntes) tracao.alternarFrente();
                yAntes = gamepad1.y;
                if (gamepad1.x) alianca = "BLUE";
                if (gamepad1.b) alianca = "RED";

                if (gamepad1.dpad_left && !esquerdaAntes)
                    ajusteGraus = Range.clip(ajusteGraus + 1.0, -20.0, 20.0);
                if (gamepad1.dpad_right && !direitaAntes)
                    ajusteGraus = Range.clip(ajusteGraus - 1.0, -20.0, 20.0);
                esquerdaAntes = gamepad1.dpad_left;
                direitaAntes = gamepad1.dpad_right;
                if (gamepad1.dpad_up && !cimaAntes) tracao.ajustarLimite(1);
                if (gamepad1.dpad_down && !baixoAntes) tracao.ajustarLimite(-1);
                cimaAntes = gamepad1.dpad_up;
                baixoAntes = gamepad1.dpad_down;

                AprilTagClusterDetection alvo = encontrarAlvo(alianca);
                tracao.atualizar(gamepad1.left_stick_y, gamepad1.right_stick_x,
                    gamepad1.left_bumper);
                double potenciaEsquerda = tracao.esquerda();
                double potenciaDireita = tracao.direita();
                String estadoMira = "manual";

                if (gamepad1.right_bumper) {
                    if (alianca.isEmpty()) {
                        estadoMira = "selecione X azul ou B vermelho";
                    } else if (alvo == null) {
                        estadoMira = "sem celula pontuavel visivel";
                    } else {
                        tracao.zerar();
                        double erro = alvo.ftcPose.bearing + ajusteGraus;
                        if (Math.abs(erro) <= TOLERANCIA_GRAUS) {
                            potenciaEsquerda = 0;
                            potenciaDireita = 0;
                            estadoMira = "ALINHADO";
                        } else {
                            double giro = Range.clip(Math.abs(erro) * GANHO_MIRA,
                                GIRO_MINIMO, GIRO_MAXIMO);
                            // Bearing positivo significa alvo a esquerda da camera.
                            // Potencia esquerda negativa e direita positiva giram a esquerda.
                            potenciaEsquerda = erro > 0 ? -giro : giro;
                            potenciaDireita = -potenciaEsquerda;
                            estadoMira = erro > 0 ? "girando a esquerda" : "girando a direita";
                        }
                    }
                }
                esquerdo.setPower(potenciaEsquerda);
                direito.setPower(potenciaDireita);

                double potenciaLancador = gamepad2.right_trigger > 0.20 ? LANCADOR : 0;
                double potenciaColetor = gamepad2.left_bumper ? -COLETOR :
                    gamepad2.left_trigger > 0.20 ? COLETOR : 0;
                lancador.setPower(potenciaLancador);
                coletor.setPower(potenciaColetor);
                if (pux != null)
                    pux.setPower(potenciaColetor > 0 ? PUX :
                        potenciaColetor < 0 ? -PUX : 0);

                if (garraPosicao != null) {
                    if (gamepad2.a) garraPosicao.setPosition(GARRA_FECHADA);
                    else if (gamepad2.b) garraPosicao.setPosition(GARRA_ABERTA);
                }
                if (garraContinua != null)
                    garraContinua.setPower(gamepad2.a ? GARRA_CONTINUA :
                        gamepad2.b ? -GARRA_CONTINUA : 0);

                telemetry.addData("Alianca", alianca.isEmpty() ? "X azul / B vermelho" : alianca);
                telemetry.addData("Mira RB", estadoMira);
                telemetry.addData("Direcao", gamepad1.left_bumper ? "PRECISA" : "NORMAL");
                telemetry.addData("Frente", tracao.frenteInvertida() ? "OPOSTA (Y alterna)" : "PADRAO (Y alterna)");
                telemetry.addData("Limite tracao", "%.0f%%", tracao.limite() * 100.0);
                telemetry.addData("Ajuste", "%.0f graus", ajusteGraus);
                if (alvo != null) {
                    telemetry.addData("Celula", alvo.metadata.name);
                    telemetry.addData("Desvio", "%.1f graus", alvo.ftcPose.bearing);
                    telemetry.addData("Distancia", "%.1f pol", alvo.ftcPose.range);
                    telemetry.addData("Roll", "%.1f graus", alvo.ftcPose.roll);
                }
                telemetry.addData("Lancador", "%.2f", potenciaLancador);
                telemetry.addData("Coletor", "%.2f", potenciaColetor);
                telemetry.update();
                idle();
            }
        } finally {
            esquerdo.setPower(0);
            direito.setPower(0);
            lancador.setPower(0);
            coletor.setPower(0);
            if (pux != null) pux.setPower(0);
            if (garraContinua != null) garraContinua.setPower(0);
            if (visionPortal != null) visionPortal.close();
        }
    }

    private AprilTagClusterDetection encontrarAlvo(String alianca) {
        if (alianca.isEmpty() || aprilTag == null) return null;
        AprilTagClusterDetection melhor = null;
        for (AprilTagDetection deteccao : aprilTag.getDetections()) {
            if (!(deteccao instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection celula = (AprilTagClusterDetection) deteccao;
            if (celula.metadata == null || celula.ftcPose == null) continue;
            if (!celula.metadata.name.toUpperCase().startsWith(alianca.substring(0, 1))) continue;
            if (Math.abs(celula.ftcPose.roll) >= 90.0) continue;
            if (melhor == null || Math.abs(celula.ftcPose.bearing) <
                    Math.abs(melhor.ftcPose.bearing)) melhor = celula;
        }
        return melhor;
    }
}

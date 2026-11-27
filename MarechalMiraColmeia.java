package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private static final double COLETOR = 1.00;
    private static final double PUX = 1.00;
    private static final double GARRA_RETORNO = 0.50;
    private static final double GARRA_BATIDA = 0.54;

    // Giro lento e somente enquanto RB do controle 1 estiver pressionado.
    private static final double GANHO_MIRA = 0.015;
    private static final double GIRO_MINIMO = 0.10;
    private static final double GIRO_MAXIMO = 0.25;
    private static final double TOLERANCIA_GRAUS = 2.0;

    private DcMotor direito;
    private DcMotor esquerdo;
    private DcMotorEx lancador;
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
        lancador = hardwareMap.get(DcMotorEx.class, "lancador");
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
        MarechalLancador controleLancador = new MarechalLancador(lancador);

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
            telemetry.addData("Garra", garraPosicao != null ? "Servo de posicao" :
                garraContinua != null ? "CONFIGURAR SERVO 0 COMO SERVO" : "NAO ENCONTRADA");
            telemetry.addLine("Controle 2: D-pad cima/baixo ajusta lancador.");
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
            boolean lancadorCimaAntes = false;
            boolean lancadorBaixoAntes = false;
            boolean garraAAntes = false;
            boolean garraBAntes = false;
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
                String estadoMira = "manual";

                if (gamepad1.right_bumper && !alianca.isEmpty() && alvo != null) {
                    double erro = alvo.ftcPose.bearing + ajusteGraus;
                    if (Math.abs(erro) <= TOLERANCIA_GRAUS) {
                        tracao.atualizarMira(0, 0);
                        estadoMira = "ALINHADO";
                    } else {
                        double giro = Range.clip(Math.abs(erro) * GANHO_MIRA,
                            GIRO_MINIMO, GIRO_MAXIMO);
                        // Bearing positivo significa alvo a esquerda da camera.
                        double esquerdaMira = erro > 0 ? -giro : giro;
                        tracao.atualizarMira(esquerdaMira, -esquerdaMira);
                        estadoMira = erro > 0 ? "girando a esquerda" : "girando a direita";
                    }
                } else {
                    tracao.atualizar(gamepad1.left_stick_y, gamepad1.right_stick_x,
                        gamepad1.left_bumper);
                    if (gamepad1.right_bumper)
                        estadoMira = alianca.isEmpty() ? "selecione X azul ou B vermelho" :
                            "sem celula pontuavel visivel";
                }
                esquerdo.setPower(tracao.esquerda());
                direito.setPower(tracao.direita());

                if (gamepad2.dpad_up && !lancadorCimaAntes) {
                    controleLancador.ajustarAlvo(1);
                }
                if (gamepad2.dpad_down && !lancadorBaixoAntes) {
                    controleLancador.ajustarAlvo(-1);
                }
                lancadorCimaAntes = gamepad2.dpad_up;
                lancadorBaixoAntes = gamepad2.dpad_down;
                controleLancador.atualizar(gamepad2.right_trigger > 0.20);
                double potenciaColetor = gamepad2.left_bumper ? -COLETOR :
                    gamepad2.left_trigger > 0.20 ? COLETOR : 0;
                coletor.setPower(potenciaColetor);
                if (pux != null)
                    pux.setPower(potenciaColetor > 0 && controleLancador.pronto() ? PUX :
                        potenciaColetor < 0 ? -PUX : 0);

                if (garraPosicao != null) {
                    if (gamepad2.a && !garraAAntes) garraPosicao.setPosition(GARRA_RETORNO);
                    else if (gamepad2.b && !garraBAntes) garraPosicao.setPosition(GARRA_BATIDA);
                }
                garraAAntes = gamepad2.a;
                garraBAntes = gamepad2.b;
                telemetry.addData("Garra", garraPosicao != null ? "Servo de posicao" :
                    garraContinua != null ? "CONFIGURAR SERVO 0 COMO SERVO" : "NAO ENCONTRADA");

                telemetry.addData("Alianca", alianca.isEmpty() ? "X azul / B vermelho" : alianca);
                telemetry.addData("Mira RB", estadoMira);
                telemetry.addData("Analogicos G1", "Y %.2f X %.2f",
                    gamepad1.left_stick_y, gamepad1.right_stick_x);
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
                telemetry.addData("Lancador alvo", "%d ticks/s", controleLancador.alvo());
                telemetry.addData("Lancador atual", "%.0f ticks/s", controleLancador.velocidade());
                telemetry.addData("Lancador estado", controleLancador.estado());
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/** Primeiro passo autonomo: gira ate a colmeia e para, sem deslocar ou disparar. */
@Autonomous(name = "Marechal - Auto Mira (teste)", group = "Equipe")
public class MarechalAutoMira extends LinearOpMode {
    private static final double TOLERANCIA_GRAUS = 2.0;
    private static final double GANHO_GIRO = 0.015;
    private static final double GIRO_MINIMO = 0.18;
    private static final double GIRO_MAXIMO = 0.25;
    private static final double TEMPO_LIMITE_SEGUNDOS = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor esquerdo = hardwareMap.get(DcMotor.class, "esquerdo");
        DcMotor direito = hardwareMap.get(DcMotor.class, "direito");
        esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        direito.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdo.setPower(0);
        direito.setPower(0);

        List<WebcamName> cameras = hardwareMap.getAll(WebcamName.class);
        if (cameras.isEmpty()) {
            telemetry.addLine("Webcam ausente da Robot Configuration.");
            telemetry.update();
            return;
        }

        AprilTagProcessor tags = new AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
            .build();
        tags.setDecimation(2);
        VisionPortal portal = new VisionPortal.Builder()
            .setCamera(cameras.get(0))
            .addProcessor(tags)
            .build();

        try {
            String alianca = "BLUE";
            while (opModeInInit()) {
                if (gamepad1.x) alianca = "BLUE";
                if (gamepad1.b) alianca = "RED";
                AprilTagClusterDetection alvo = encontrarAlvo(tags, alianca);
                telemetry.addLine("AUTO MIRA: X azul, B vermelho na INIT.");
                telemetry.addLine("START gira no lugar; nao avanca nem dispara.");
                telemetry.addData("Alianca", alianca);
                telemetry.addData("Celula", alvo != null ? alvo.metadata.name : "NAO VISIVEL");
                if (alvo != null)
                    telemetry.addData("Desvio", "%.1f graus", alvo.ftcPose.bearing);
                telemetry.update();
                idle();
            }
            if (isStopRequested()) return;

            ElapsedTime tempo = new ElapsedTime();
            boolean alinhado = false;
            while (opModeIsActive() && !alinhado && tempo.seconds() < TEMPO_LIMITE_SEGUNDOS) {
                AprilTagClusterDetection alvo = encontrarAlvo(tags, alianca);
                double esquerda = 0;
                double direita = 0;
                String estado = "AGUARDANDO CELULA";
                if (alvo != null) {
                    double erro = alvo.ftcPose.bearing;
                    if (Math.abs(erro) <= TOLERANCIA_GRAUS) {
                        alinhado = true;
                        estado = "ALINHADO";
                    } else {
                        double giro = Range.clip(Math.abs(erro) * GANHO_GIRO,
                            GIRO_MINIMO, GIRO_MAXIMO);
                        esquerda = erro > 0 ? -giro : giro;
                        direita = -esquerda;
                        estado = "GIRANDO";
                    }
                    telemetry.addData("Celula", alvo.metadata.name);
                    telemetry.addData("Desvio", "%.1f graus", erro);
                    telemetry.addData("Distancia", "%.1f pol", alvo.ftcPose.range);
                }
                esquerdo.setPower(esquerda);
                direito.setPower(direita);
                telemetry.addData("Auto Mira", estado);
                telemetry.addData("Tempo", "%.1f / %.1f s",
                    tempo.seconds(), TEMPO_LIMITE_SEGUNDOS);
                telemetry.update();
                idle();
            }

            esquerdo.setPower(0);
            direito.setPower(0);
            while (opModeIsActive() && tempo.seconds() < 30.0) {
                telemetry.addData("Auto Mira", alinhado ? "ALINHADO: PARADO" :
                    "TEMPO ESGOTADO: PARADO");
                telemetry.update();
                idle();
            }
        } finally {
            esquerdo.setPower(0);
            direito.setPower(0);
            portal.close();
        }
    }

    private AprilTagClusterDetection encontrarAlvo(AprilTagProcessor tags, String alianca) {
        AprilTagClusterDetection melhor = null;
        for (AprilTagDetection deteccao : tags.getDetections()) {
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

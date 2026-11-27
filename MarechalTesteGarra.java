package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Ajuste isolado da garra: nao controla rodas, coletor ou lancador. */
@TeleOp(name = "Marechal - Teste Garra", group = "Equipe")
public class MarechalTesteGarra extends LinearOpMode {
    private static final int POSICAO_MINIMA = 30;
    private static final int POSICAO_MAXIMA = 70;
    private static final int PASSO = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo garra = hardwareMap.tryGet(Servo.class, "garra");
        if (garra == null) {
            telemetry.addLine("Garra nao encontrada como Servo de posicao.");
            telemetry.addLine("Configure SERVO 0 garra como Servo.");
            telemetry.update();
            return;
        }

        telemetry.addLine("Teste isolado: somente a garra pode mover.");
        telemetry.addLine("Controle 2: A diminui, B aumenta 0.02 por toque.");
        telemetry.addLine("X volta ao centro (0.50).");
        telemetry.update();
        waitForStart();

        int posicao = 50;
        boolean aAntes = false;
        boolean bAntes = false;
        boolean xAntes = false;
        garra.setPosition(posicao / 100.0);
        int ultimaComandada = posicao;

        while (opModeIsActive()) {
            if (gamepad2.x && !xAntes) posicao = 50;
            else if (gamepad2.a && !aAntes)
                posicao = Math.max(POSICAO_MINIMA, posicao - PASSO);
            else if (gamepad2.b && !bAntes)
                posicao = Math.min(POSICAO_MAXIMA, posicao + PASSO);

            if (posicao != ultimaComandada) {
                garra.setPosition(posicao / 100.0);
                ultimaComandada = posicao;
            }
            aAntes = gamepad2.a;
            bAntes = gamepad2.b;
            xAntes = gamepad2.x;

            telemetry.addData("POSICAO DA GARRA", "%.2f", posicao / 100.0);
            telemetry.addLine("A diminui | B aumenta | X centro");
            telemetry.update();
            idle();
        }
    }
}

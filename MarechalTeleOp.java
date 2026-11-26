package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp inicial do robo FTC Marechal.
 * Os nomes devem coincidir com Robot Configuration.
 * Portas confirmadas: MOTOR 0 direito, MOTOR 1 esquerdo,
 * MOTOR 2 lancador, MOTOR 3 coletor/pegador.
 * SERVO 0 garra, SERVO 1 pux (rotacao continua).
 * Teste as rodas suspensas e os mecanismos sem bolas.
 */
@TeleOp(name = "Marechal - TeleOp", group = "Equipe")
public class MarechalTeleOp extends LinearOpMode {

    private DcMotor direito;
    private DcMotor esquerdo;
    private DcMotor lancador;
    private DcMotor coletor;
    private Servo garraPosicao;
    private CRServo garraContinua;
    private CRServo pux;

    // Potencias iniciais de teste; ajustar conforme o mecanismo real.
    private static final double LANCADOR = 0.85;
    private static final double COLETOR = 0.70;
    private static final double PUX = 1.00;
    private static final double GARRA_CONTINUA = 0.25;

    // POSICOES DE TESTE: ajustar com a garra sem carga.
    private static final double GARRA_FECHADA = 0.45;
    private static final double GARRA_ABERTA = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {
        direito = hardwareMap.get(DcMotor.class, "direito");
        esquerdo = hardwareMap.get(DcMotor.class, "esquerdo");
        lancador = hardwareMap.get(DcMotor.class, "lancador");
        coletor = hardwareMap.get(DcMotor.class, "coletor");
        garraPosicao = hardwareMap.tryGet(Servo.class, "garra");
        garraContinua = hardwareMap.tryGet(CRServo.class, "garra");
        pux = hardwareMap.tryGet(CRServo.class, "pux");

        // Inverter se os testes mostrarem sentidos incorretos.
        direito.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);

        direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lancador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        direito.setPower(0);
        esquerdo.setPower(0);
        lancador.setPower(0);
        coletor.setPower(0);
        if (pux != null) pux.setPower(0);
        if (garraContinua != null) garraContinua.setPower(0);

        telemetry.addLine("Pronto: teste com rodas suspensas.");
        telemetry.addLine("Controle 1: Y alterna o lado da frente.");
        telemetry.addLine("Confirme os nomes no Robot Configuration.");
        telemetry.addData("Garra", garraPosicao != null ? "Servo" :
            garraContinua != null ? "Continuous Servo" : "NAO ENCONTRADA");
        telemetry.addData("Pux", pux != null ? "Continuous Servo" : "NAO ENCONTRADO");
        telemetry.update();

        waitForStart();

        try {
            MarechalTracao tracao = new MarechalTracao();
            boolean cimaAntes = false;
            boolean baixoAntes = false;
            boolean yAntes = false;
            while (opModeIsActive()) {
                if (gamepad1.y && !yAntes) tracao.alternarFrente();
                yAntes = gamepad1.y;
                if (gamepad1.dpad_up && !cimaAntes) tracao.ajustarLimite(1);
                if (gamepad1.dpad_down && !baixoAntes) tracao.ajustarLimite(-1);
                cimaAntes = gamepad1.dpad_up;
                baixoAntes = gamepad1.dpad_down;

                // Esquerdo Y acelera/re; direito X faz curvas.
                tracao.atualizar(gamepad1.left_stick_y, gamepad1.right_stick_x,
                    gamepad1.left_bumper);
                double e = tracao.esquerda();
                double d = tracao.direita();
                esquerdo.setPower(e);
                direito.setPower(d);

                // Controle 2: RT dispara, LT coleta e LB reverte.
                double potenciaLancador =
                    gamepad2.right_trigger > 0.20 ? LANCADOR : 0.0;
                double potenciaColetor = 0.0;

                if (gamepad2.left_bumper) {
                    potenciaColetor = -COLETOR;
                } else if (gamepad2.left_trigger > 0.20) {
                    potenciaColetor = COLETOR;
                }
                lancador.setPower(potenciaLancador);
                coletor.setPower(potenciaColetor);
                double potenciaPux = 0.0;
                if (potenciaColetor > 0.0) {
                    potenciaPux = PUX;
                } else if (potenciaColetor < 0.0) {
                    potenciaPux = -PUX;
                }
                if (pux != null) pux.setPower(potenciaPux);

                // Garra de posicao ou continua, conforme Robot Configuration.
                if (garraPosicao != null) {
                    if (gamepad2.a) {
                        garraPosicao.setPosition(GARRA_FECHADA);
                    } else if (gamepad2.b) {
                        garraPosicao.setPosition(GARRA_ABERTA);
                    }
                }
                if (garraContinua != null) {
                    double potenciaGarra = gamepad2.a ? GARRA_CONTINUA :
                        gamepad2.b ? -GARRA_CONTINUA : 0.0;
                    garraContinua.setPower(potenciaGarra);
                }

                telemetry.addData("Esquerdo", "%.2f", e);
                telemetry.addData("Direito", "%.2f", d);
                telemetry.addData("Direcao", gamepad1.left_bumper ? "PRECISA" : "NORMAL");
                telemetry.addData("Frente", tracao.frenteInvertida() ? "OPOSTA (Y alterna)" : "PADRAO (Y alterna)");
                telemetry.addData("Limite tracao", "%.0f%%", tracao.limite() * 100.0);
                telemetry.addData("Lancador", "%.2f", potenciaLancador);
                telemetry.addData("Coletor", "%.2f", potenciaColetor);
                telemetry.addData("Pux", pux != null ? String.format("%.2f", potenciaPux) : "NAO ENCONTRADO");
                telemetry.addData("Garra", garraPosicao != null ? "Servo" :
                    garraContinua != null ? "Continuous Servo" : "NAO ENCONTRADA");
                telemetry.addData("Entradas G2", "LT %.2f LB %b A %b B %b",
                    gamepad2.left_trigger, gamepad2.left_bumper, gamepad2.a, gamepad2.b);
                telemetry.update();
                idle();
            }
        } finally {
            direito.setPower(0);
            esquerdo.setPower(0);
            lancador.setPower(0);
            coletor.setPower(0);
            if (pux != null) pux.setPower(0);
            if (garraContinua != null) garraContinua.setPower(0);
        }
    }
}

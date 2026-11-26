package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Base SEGURA para a fase autonoma de 30 segundos.
 * Nao executa trajetos sem antes calibrar encoders e pista.
 * Portas confirmadas: MOTOR 0 direito e MOTOR 1 esquerdo.
 */
@Autonomous(name = "Marechal - Auto Base", group = "Equipe")
public class MarechalAutoBase extends LinearOpMode {

    private DcMotor direito;
    private DcMotor esquerdo;

    // Iniciar desativado. Ativar APENAS depois da calibracao.
    private static final boolean HABILITAR_TESTE = false;
    private static final int TICKS_AVANCO_TESTE = 0;
    private static final double POTENCIA_TESTE = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {
        direito = hardwareMap.get(DcMotor.class, "direito");
        esquerdo = hardwareMap.get(DcMotor.class, "esquerdo");

        direito.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        direito.setPower(0);
        esquerdo.setPower(0);

        telemetry.addLine("Auto Base: SEM MOVIMENTO por padrao.");
        telemetry.addLine("Calibrar distancias e area de teste.");
        telemetry.update();
        waitForStart();

        ElapsedTime tempo = new ElapsedTime();
        tempo.reset();

        try {
            if (HABILITAR_TESTE && TICKS_AVANCO_TESTE > 0) {
                // Use somente apos testar ambos os encoders.
                direito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                esquerdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                direito.setTargetPosition(TICKS_AVANCO_TESTE);
                esquerdo.setTargetPosition(TICKS_AVANCO_TESTE);
                direito.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                esquerdo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                direito.setPower(POTENCIA_TESTE);
                esquerdo.setPower(POTENCIA_TESTE);

                // Limite de tempo de seguranca para o teste curto.
                while (opModeIsActive() && tempo.seconds() < 3.0 &&
                       (direito.isBusy() || esquerdo.isBusy())) {
                    telemetry.addData("Tempo", "%.1fs", tempo.seconds());
                    telemetry.update();
                    idle();
                }
                direito.setPower(0);
                esquerdo.setPower(0);
            }

            // Reserva a fase autonoma; nao inventa trajetos.
            while (opModeIsActive() && tempo.seconds() < 30.0) {
                telemetry.addData("Autonomo", "%.1f / 30 s", tempo.seconds());
                telemetry.addLine("Aguardando plano calibrado de pista.");
                telemetry.update();
                idle();
            }
        } finally {
            direito.setPower(0);
            esquerdo.setPower(0);
        }
    }
}

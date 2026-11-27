package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Dirija o percurso uma vez; X inicia e X termina e salva a gravacao. */
@TeleOp(name = "Marechal - Gravar Percurso", group = "Equipe")
public class MarechalGravarPercurso extends LinearOpMode {
    private static final long INTERVALO_AMOSTRA_NS = 50_000_000L;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor esquerdo = hardwareMap.get(DcMotor.class, "esquerdo");
        DcMotor direito = hardwareMap.get(DcMotor.class, "direito");
        esquerdo.setDirection(DcMotorSimple.Direction.REVERSE);
        direito.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        direito.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        esquerdo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        direito.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdo.setPower(0);
        direito.setPower(0);
        DcMotor coletor = hardwareMap.get(DcMotor.class, "coletor");
        CRServo pux = hardwareMap.get(CRServo.class, "pux");
        MarechalLancador lancador = new MarechalLancador(
            hardwareMap.get(DcMotorEx.class, "lancador"));
        coletor.setPower(0);
        pux.setPower(0);

        int slot = 1;
        boolean esquerdaAntes = false, direitaAntes = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_left && !esquerdaAntes)
                slot = Math.max(1, slot - 1);
            if (gamepad1.dpad_right && !direitaAntes)
                slot = Math.min(MarechalGravacao.MAX_SLOTS, slot + 1);
            esquerdaAntes = gamepad1.dpad_left;
            direitaAntes = gamepad1.dpad_right;
            telemetry.addLine("GRAVAR: D-pad esquerda/direita escolhe o slot.");
            telemetry.addData("Slot selecionado", slot + " / " + MarechalGravacao.MAX_SLOTS);
            telemetry.addData("Situacao", MarechalGravacao.existe(hardwareMap.appContext, slot)
                ? "OCUPADO: gravar substitui este slot" : "VAZIO");
            telemetry.addLine("START ativa; X comeca a gravar; X salva.");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        List<MarechalGravacao.Amostra> amostras = new ArrayList<MarechalGravacao.Amostra>();
        MarechalTracao tracao = new MarechalTracao();
        boolean gravando = false;
        boolean xAntes = false;
        boolean yAntes = false;
        long inicioNs = 0;
        long ultimaAmostraNs = 0;
        int inicioEsquerdo = 0, inicioDireito = 0;
        boolean cimaAntes = false, baixoAntes = false;
        String estado = "AGUARDANDO X PARA INICIAR";

        try {
            while (opModeIsActive()) {
                long agora = System.nanoTime();
                if (gamepad1.x && !xAntes) {
                    if (!gravando) {
                        amostras.clear();
                        tracao = new MarechalTracao();
                        inicioNs = agora;
                        ultimaAmostraNs = agora;
                        inicioEsquerdo = esquerdo.getCurrentPosition();
                        inicioDireito = direito.getCurrentPosition();
                        amostras.add(new MarechalGravacao.Amostra(0, 0, 0,
                            0, 0, false, lancador.alvo(), 0, 0));
                        gravando = true;
                        estado = "GRAVANDO: DIRIJA O PERCURSO";
                    } else {
                        gravando = false;
                        esquerdo.setPower(0);
                        direito.setPower(0);
                        coletor.setPower(0);
                        pux.setPower(0);
                        lancador.atualizar(false);
                        estado = salvar(amostras, inicioNs, agora,
                            esquerdo.getCurrentPosition() - inicioEsquerdo,
                            direito.getCurrentPosition() - inicioDireito,
                            lancador.alvo(), slot);
                    }
                }
                xAntes = gamepad1.x;
                if (gravando && gamepad1.y && !yAntes)
                    tracao.alternarFrente();
                yAntes = gamepad1.y;

                if (gravando) {
                    double decorrido = (agora - inicioNs) / 1.0e9;
                    if (decorrido >= MarechalGravacao.DURACAO_MAXIMA) {
                        gravando = false;
                        esquerdo.setPower(0);
                        direito.setPower(0);
                        coletor.setPower(0);
                        pux.setPower(0);
                        lancador.atualizar(false);
                        estado = salvar(amostras, inicioNs,
                            inicioNs + (long) (MarechalGravacao.DURACAO_MAXIMA * 1.0e9),
                            esquerdo.getCurrentPosition() - inicioEsquerdo,
                            direito.getCurrentPosition() - inicioDireito,
                            lancador.alvo(), slot);
                    } else {
                        if (gamepad2.dpad_up && !cimaAntes) lancador.ajustarAlvo(1);
                        if (gamepad2.dpad_down && !baixoAntes) lancador.ajustarAlvo(-1);
                        cimaAntes = gamepad2.dpad_up;
                        baixoAntes = gamepad2.dpad_down;
                        tracao.atualizar(gamepad1.left_stick_y,
                            gamepad1.right_stick_x, gamepad1.left_bumper);
                        esquerdo.setPower(tracao.esquerda());
                        direito.setPower(tracao.direita());
                        boolean atirar = gamepad2.right_trigger > 0.20;
                        lancador.atualizar(atirar);
                        double coletorPedido = gamepad2.left_bumper ? -1.0 :
                            (gamepad2.left_trigger > 0.20 ? 1.0 : 0.0);
                        double puxPedido = coletorPedido;
                        coletor.setPower(coletorPedido);
                        pux.setPower(puxPedido > 0 && !lancador.pronto() ? 0 : puxPedido);
                        if (agora - ultimaAmostraNs >= INTERVALO_AMOSTRA_NS) {
                            amostras.add(new MarechalGravacao.Amostra(decorrido,
                                tracao.esquerda(), tracao.direita(),
                                esquerdo.getCurrentPosition() - inicioEsquerdo,
                                direito.getCurrentPosition() - inicioDireito,
                                atirar, lancador.alvo(), coletorPedido, puxPedido));
                            ultimaAmostraNs = agora;
                        }
                    }
                } else {
                    esquerdo.setPower(0);
                    direito.setPower(0);
                    coletor.setPower(0);
                    pux.setPower(0);
                    lancador.atualizar(false);
                }

                telemetry.addData("Estado", estado);
                telemetry.addData("Slot", slot);
                telemetry.addLine("Controle 1: esquerdo frente/re, direito vira.");
                telemetry.addData("Frente", tracao.frenteInvertida() ?
                    "OPOSTA (Y alterna)" : "PADRAO (Y alterna)");
                telemetry.addLine("X: comecar / terminar e salvar");
                telemetry.addLine("Controle 2: RT lancador, LT alimenta, LB reverte, DPAD +/-50.");
                telemetry.addData("Lancador", "%d alvo / %.0f atual / %s",
                    lancador.alvo(), lancador.velocidade(), lancador.estado());
                if (gravando) {
                    telemetry.addData("Tempo gravado", "%.1f / %.0f s",
                        (agora - inicioNs) / 1.0e9,
                        MarechalGravacao.DURACAO_MAXIMA);
                }
                telemetry.update();
                idle();
            }
        } finally {
            esquerdo.setPower(0);
            direito.setPower(0);
            coletor.setPower(0);
            pux.setPower(0);
            lancador.parar();
        }
    }

    private String salvar(List<MarechalGravacao.Amostra> amostras,
                          long inicioNs, long fimNs, int ticksEsquerdo,
                          int ticksDireito, int alvoLancador, int slot) {
        double duracao = (fimNs - inicioNs) / 1.0e9;
        amostras.add(new MarechalGravacao.Amostra(duracao, 0, 0,
            ticksEsquerdo, ticksDireito, false, alvoLancador, 0, 0));
        try {
            MarechalGravacao.salvar(hardwareMap.appContext,
                new MarechalGravacao.Trajeto(amostras, duracao), slot);
            return String.format(Locale.US, "SLOT %d SALVO: %.1f s.", slot, duracao);
        } catch (IOException erro) {
            return "NAO SALVO: " + erro.getMessage();
        }
    }
}

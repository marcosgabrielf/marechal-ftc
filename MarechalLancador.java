package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Controla a velocidade do lancador e autoriza a alimentacao pelo servo pux. */
final class MarechalLancador {
    private static final int ALVO_INICIAL = 1300;
    private static final int ALVO_MINIMO = 700;
    private static final int ALVO_MAXIMO = 2200;
    private static final int PASSO = 50;
    private static final double ENTRADA_ABAIXO = 50.0;
    private static final double ENTRADA_ACIMA = 100.0;
    private static final double SAIDA_ABAIXO = 200.0;
    private static final double SAIDA_ACIMA = 250.0;
    private static final long TEMPO_ESTAVEL_NS = 200_000_000L;
    private static final long TEMPO_FORA_NS = 150_000_000L;
    private static final long TEMPO_ENCODER_NS = 1_500_000_000L;
    private static final long TEMPO_SEM_SINAL_NS = 600_000_000L;

    private final DcMotorEx motor;
    private int alvo = ALVO_INICIAL;
    private int posicaoInicial;
    private long inicioVerificacaoNs;
    private long inicioEstavelNs;
    private long inicioForaNs;
    private long inicioSemSinalNs;
    private boolean encoderValidado;
    private boolean falhaEncoder;
    private boolean pronto;
    private double velocidade;
    private String estado = "PARADO";

    MarechalLancador(DcMotorEx motor) {
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    void ajustarAlvo(int direcao) {
        alvo = Math.max(ALVO_MINIMO, Math.min(ALVO_MAXIMO, alvo + direcao * PASSO));
        pronto = false;
        inicioEstavelNs = 0;
        inicioForaNs = 0;
    }

    void atualizar(boolean acionado) {
        long agora = System.nanoTime();
        if (!acionado) {
            motor.setPower(0);
            inicioVerificacaoNs = 0;
            inicioEstavelNs = 0;
            inicioForaNs = 0;
            inicioSemSinalNs = 0;
            encoderValidado = false;
            falhaEncoder = false;
            pronto = false;
            velocidade = 0;
            estado = "PARADO";
            return;
        }

        if (falhaEncoder) {
            motor.setPower(0);
            pronto = false;
            estado = "ENCODER SEM SINAL; solte RT";
            return;
        }

        if (!encoderValidado) {
            if (inicioVerificacaoNs == 0) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                posicaoInicial = motor.getCurrentPosition();
                inicioVerificacaoNs = agora;
                motor.setPower(0.50);
            }
            velocidade = Math.abs(motor.getVelocity());
            if (Math.abs(motor.getCurrentPosition() - posicaoInicial) >= 10 &&
                    velocidade >= 100.0) {
                encoderValidado = true;
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocity(alvo);
                estado = "ACELERANDO";
            } else if (agora - inicioVerificacaoNs >= TEMPO_ENCODER_NS) {
                motor.setPower(0);
                falhaEncoder = true;
                estado = "ENCODER SEM SINAL; solte RT";
            } else {
                estado = "VERIFICANDO ENCODER";
            }
            pronto = false;
            return;
        }

        motor.setVelocity(alvo);
        velocidade = Math.abs(motor.getVelocity());
        if (velocidade < 50.0) {
            if (inicioSemSinalNs == 0) inicioSemSinalNs = agora;
            if (agora - inicioSemSinalNs >= TEMPO_SEM_SINAL_NS) {
                motor.setPower(0);
                falhaEncoder = true;
                pronto = false;
                estado = "ENCODER SEM SINAL; solte RT";
                return;
            }
        } else {
            inicioSemSinalNs = 0;
        }

        if (pronto) {
            // A bola causa uma queda breve de velocidade: evitar liga/desliga do pux.
            boolean podeManter = velocidade >= alvo - SAIDA_ABAIXO &&
                velocidade <= alvo + SAIDA_ACIMA;
            if (podeManter) {
                inicioForaNs = 0;
            } else if (inicioForaNs == 0) {
                inicioForaNs = agora;
            } else if (agora - inicioForaNs >= TEMPO_FORA_NS) {
                pronto = false;
                inicioEstavelNs = 0;
                inicioForaNs = 0;
            }
        } else {
            boolean naFaixa = velocidade >= alvo - ENTRADA_ABAIXO &&
                velocidade <= alvo + ENTRADA_ACIMA;
            if (naFaixa) {
                if (inicioEstavelNs == 0) inicioEstavelNs = agora;
                pronto = agora - inicioEstavelNs >= TEMPO_ESTAVEL_NS;
            } else {
                inicioEstavelNs = 0;
            }
        }
        estado = pronto ? "PRONTO PARA ALIMENTAR" : "ACELERANDO";
    }

    int alvo() { return alvo; }
    double velocidade() { return velocidade; }
    boolean pronto() { return pronto; }
    String estado() { return estado; }

    void parar() { motor.setPower(0); }
}

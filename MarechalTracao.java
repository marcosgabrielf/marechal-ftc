package org.firstinspires.ftc.teamcode;

/** Mistura de aceleracao e curva para os dois motores de tracao. */
final class MarechalTracao {
    private static final double POTENCIA_INICIAL = 0.85;
    private static final double POTENCIA_MINIMA = 0.40;
    private static final double POTENCIA_MAXIMA = 1.00;
    private static final double PASSO_POTENCIA = 0.05;
    private static final double ZONA_MORTA = 0.05;
    private static final double ACELERACAO_POR_SEGUNDO = 4.5;
    private static final double DESACELERACAO_POR_SEGUNDO = 6.0;

    private double limite = POTENCIA_INICIAL;
    private boolean frenteInvertida;
    private double esquerda;
    private double direita;
    private long ultimoTempoNs;

    void atualizar(double joystickEsquerdoY, double joystickDireitoX,
                   boolean modoPreciso) {
        long agora = System.nanoTime();
        double intervalo = ultimoTempoNs == 0 ? 0.04 :
            Math.min(0.10, Math.max(0.0, (agora - ultimoTempoNs) / 1.0e9));
        ultimoTempoNs = agora;

        // Apenas Y do esquerdo e X do direito: diagonais leves nao alteram o comando.
        double acelerador = curva(-joystickEsquerdoY, 0.15) * limite;
        if (frenteInvertida) acelerador = -acelerador;
        double giro = curva(joystickDireitoX, 0.25) * limite * 0.75;
        if (modoPreciso) {
            acelerador *= 0.55;
            giro *= 0.55;
        }

        if (acelerador == 0.0 && giro == 0.0) {
            esquerda = 0.0;
            direita = 0.0;
            return;
        }

        double alvoEsquerda = acelerador + giro;
        double alvoDireita = acelerador - giro;
        double maior = Math.max(Math.abs(alvoEsquerda), Math.abs(alvoDireita));
        if (maior > limite) {
            alvoEsquerda *= limite / maior;
            alvoDireita *= limite / maior;
        }

        esquerda = aproximar(esquerda, alvoEsquerda, intervalo);
        direita = aproximar(direita, alvoDireita, intervalo);
    }

    double esquerda() { return esquerda; }
    double direita() { return direita; }
    double limite() { return limite; }
    boolean frenteInvertida() { return frenteInvertida; }

    void alternarFrente() {
        frenteInvertida = !frenteInvertida;
        zerar();
    }

    void ajustarLimite(int direcao) {
        limite = Math.max(POTENCIA_MINIMA,
            Math.min(POTENCIA_MAXIMA,
                Math.round((limite + direcao * PASSO_POTENCIA) * 100.0) / 100.0));
    }

    void zerar() {
        esquerda = 0.0;
        direita = 0.0;
        ultimoTempoNs = System.nanoTime();
    }

    private static double curva(double entrada, double pesoCubico) {
        double valor = Math.abs(entrada);
        if (valor <= ZONA_MORTA) return 0.0;
        double ajustado = (valor - ZONA_MORTA) / (1.0 - ZONA_MORTA);
        double suave = (1.0 - pesoCubico) * ajustado +
            pesoCubico * ajustado * ajustado * ajustado;
        return Math.copySign(suave, entrada);
    }

    private static double aproximar(double atual, double alvo, double intervalo) {
        double taxa = Math.abs(alvo) < Math.abs(atual) ?
            DESACELERACAO_POR_SEGUNDO : ACELERACAO_POR_SEGUNDO;
        double passo = taxa * intervalo;
        if (alvo > atual) return Math.min(alvo, atual + passo);
        return Math.max(alvo, atual - passo);
    }
}

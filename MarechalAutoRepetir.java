package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.io.IOException;
import java.util.List;

/** Repete percurso e mecanismos, corrigindo cada roda pelo proprio encoder. */
@Autonomous(name = "Marechal - Auto Repetir", group = "Equipe")
public class MarechalAutoRepetir extends LinearOpMode {
    private static final double GANHO_TICK = 0.002;
    private static final double CORRECAO_MAXIMA = 0.18;
    private static final int FOLGA_TICKS = 15;
    private static final double TOLERANCIA_GRAUS = 2.0;
    private static final double GANHO_MIRA = 0.015;
    private static final double GIRO_MINIMO = 0.10;
    private static final double GIRO_MAXIMO = 0.23;
    private static final double LIMITE_MIRA_S = 3.0;

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
        AprilTagProcessor tags = null;
        VisionPortal portal = null;
        String estadoCamera = "WEBCAM AUSENTE";
        List<WebcamName> cameras = hardwareMap.getAll(WebcamName.class);
        if (!cameras.isEmpty()) {
            try {
                tags = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                    .build();
                tags.setDecimation(2);
                portal = new VisionPortal.Builder().setCamera(cameras.get(0))
                    .addProcessor(tags).build();
                estadoCamera = "INICIANDO";
            } catch (RuntimeException erro) {
                estadoCamera = "FALHA NA WEBCAM";
                if (portal != null) { portal.close(); portal = null; }
                tags = null;
            }
        }

        MarechalGravacao.Trajeto trajeto = null;
        String erroLeitura = null;

        try {
            String alianca = "BLUE";
            int slot = 1;
            int slotCarregado = 0;
            boolean slotEsquerdaAntes = false, slotDireitaAntes = false;
            while (opModeInInit()) {
                if (gamepad1.x) alianca = "BLUE";
                if (gamepad1.b) alianca = "RED";
                if (gamepad1.dpad_left && !slotEsquerdaAntes)
                    slot = Math.max(1, slot - 1);
                if (gamepad1.dpad_right && !slotDireitaAntes)
                    slot = Math.min(MarechalGravacao.MAX_SLOTS, slot + 1);
                slotEsquerdaAntes = gamepad1.dpad_left;
                slotDireitaAntes = gamepad1.dpad_right;
                if (slot != slotCarregado) {
                    slotCarregado = slot;
                    trajeto = null;
                    erroLeitura = null;
                    try {
                        trajeto = MarechalGravacao.carregar(hardwareMap.appContext, slot);
                    } catch (IOException erro) {
                        erroLeitura = erro.getMessage();
                    }
                }
                AprilTagClusterDetection alvoInit = encontrarAlvo(tags, alianca);
                telemetry.addLine("INIT: X azul, B vermelho. START inicia.");
                telemetry.addLine("D-pad esquerda/direita: escolhe a gravacao.");
                telemetry.addData("Slot selecionado", slot + " / " + MarechalGravacao.MAX_SLOTS);
                telemetry.addData("Alianca", alianca);
                telemetry.addData("Webcam", tags == null ? estadoCamera :
                    alvoInit == null ? "SEM TAG VISIVEL" : alvoInit.metadata.name);
                if (trajeto == null) {
                    telemetry.addLine("SLOT VAZIO: use Gravar Percurso.");
                    if (erroLeitura != null) telemetry.addData("Erro", erroLeitura);
                } else {
                    telemetry.addLine("Posicione no mesmo ponto e direcao da gravacao.");
                    telemetry.addData("Duracao", decimal(trajeto.duracao) + " s");
                    telemetry.addData("Amostras", trajeto.amostras.size());
                    telemetry.addData("Correcao dos encoders",
                        trajeto.temEncoders() ? "ATIVA" : "SEM MOVIMENTO MEDIDO");
                }
                telemetry.update();
                idle();
            }
            if (isStopRequested()) return;

            ElapsedTime tempo = new ElapsedTime();
            String resultado = trajeto == null ? "SLOT VAZIO: PARADO" :
                "SLOT " + slot + " CONCLUIDO";
            if (trajeto != null) {
                String estadoMira = "SEM TAG VISIVEL";
                if (encontrarAlvo(tags, alianca) != null)
                    estadoMira = alinhar(tags, alianca, esquerdo, direito,
                        coletor, pux, lancador, false);
                int inicioEsquerdo = esquerdo.getCurrentPosition();
                int inicioDireito = direito.getCurrentPosition();
                int indice = 0;
                double pausaMira = 0;
                boolean alimentacaoAntes = false;
                tempo.reset();
                while (opModeIsActive() && tempo.seconds() - pausaMira < trajeto.duracao &&
                        tempo.seconds() < MarechalGravacao.DURACAO_MAXIMA + 10.0) {
                    double agora = tempo.seconds() - pausaMira;
                    while (indice + 1 < trajeto.amostras.size() - 1 &&
                            trajeto.amostras.get(indice + 1).tempo <= agora)
                        indice++;
                    MarechalGravacao.Amostra a = trajeto.amostras.get(indice);
                    MarechalGravacao.Amostra b = trajeto.amostras.get(indice + 1);
                    double intervalo = b.tempo - a.tempo;
                    double fracao = intervalo <= 0 ? 0 :
                        limitar((agora - a.tempo) / intervalo, 0, 1);
                    double baseEsquerda = misturar(a.esquerda, b.esquerda, fracao);
                    double baseDireita = misturar(a.direita, b.direita, fracao);
                    double alvoEsquerdo = misturar(a.ticksEsquerdo, b.ticksEsquerdo, fracao);
                    double alvoDireito = misturar(a.ticksDireito, b.ticksDireito, fracao);
                    boolean alimentando = a.pux > 0;
                    if (alimentando && !alimentacaoAntes &&
                            encontrarAlvo(tags, alianca) != null) {
                        double inicioPausa = tempo.seconds();
                        estadoMira = alinhar(tags, alianca, esquerdo, direito,
                            coletor, pux, lancador, a.lancadorAtivo);
                        pausaMira += tempo.seconds() - inicioPausa;
                        // O giro da mira muda os encoders: continue o trajeto
                        // a partir desta nova orientacao, sem desfazer o alinhamento.
                        inicioEsquerdo = esquerdo.getCurrentPosition() -
                            (int) Math.round(alvoEsquerdo);
                        inicioDireito = direito.getCurrentPosition() -
                            (int) Math.round(alvoDireito);
                        alimentacaoAntes = true;
                        continue;
                    }
                    alimentacaoAntes = alimentando;
                    double erroEsquerdo = alvoEsquerdo -
                        (esquerdo.getCurrentPosition() - inicioEsquerdo);
                    double erroDireito = alvoDireito -
                        (direito.getCurrentPosition() - inicioDireito);
                    double potenciaEsquerda = baseEsquerda;
                    double potenciaDireita = baseDireita;
                    if (trajeto.polaridadeEsquerda != 0)
                        potenciaEsquerda += correcao(erroEsquerdo) * trajeto.polaridadeEsquerda;
                    if (trajeto.polaridadeDireita != 0)
                        potenciaDireita += correcao(erroDireito) * trajeto.polaridadeDireita;
                    esquerdo.setPower(limitar(potenciaEsquerda, -1, 1));
                    direito.setPower(limitar(potenciaDireita, -1, 1));

                    int alvo = a.alvoLancador;
                    if (alvo != lancador.alvo())
                        lancador.ajustarAlvo((alvo - lancador.alvo()) / 50);
                    lancador.atualizar(a.lancadorAtivo);
                    coletor.setPower(a.coletor);
                    pux.setPower(a.pux > 0 && !lancador.pronto() ? 0 : a.pux);

                    telemetry.addData("Repetindo", decimal(agora) + " / " +
                        decimal(trajeto.duracao) + " s");
                    telemetry.addData("Erro rodas E/D", Math.round(erroEsquerdo) +
                        " / " + Math.round(erroDireito) + " ticks");
                    telemetry.addData("Lancador", lancador.alvo() + " / " +
                        Math.round(lancador.velocidade()) + " / " + lancador.estado());
                    telemetry.addData("Mira webcam", estadoMira);
                    telemetry.update();
                    idle();
                }
            }

            esquerdo.setPower(0);
            direito.setPower(0);
            coletor.setPower(0);
            pux.setPower(0);
            lancador.atualizar(false);
            while (opModeIsActive() && tempo.seconds() < 30.0) {
                telemetry.addData("Auto Repetir", resultado);
                telemetry.update();
                idle();
            }
        } finally {
            esquerdo.setPower(0);
            direito.setPower(0);
            coletor.setPower(0);
            pux.setPower(0);
            lancador.parar();
            if (portal != null) portal.close();
        }
    }

    private String alinhar(AprilTagProcessor tags, String alianca,
                          DcMotor esquerdo, DcMotor direito, DcMotor coletor,
                          CRServo pux, MarechalLancador lancador,
                          boolean manterLancador) {
        ElapsedTime relogio = new ElapsedTime();
        double dentroDesde = -1;
        String estado = "TAG PERDIDA";
        coletor.setPower(0);
        pux.setPower(0);
        while (opModeIsActive() && relogio.seconds() < LIMITE_MIRA_S) {
            AprilTagClusterDetection alvo = encontrarAlvo(tags, alianca);
            if (alvo == null) break;
            double erro = alvo.ftcPose.bearing;
            double giro = 0;
            if (Math.abs(erro) <= TOLERANCIA_GRAUS) {
                if (dentroDesde < 0) dentroDesde = relogio.seconds();
                if (relogio.seconds() - dentroDesde >= 0.20) {
                    estado = "ALINHADO";
                    break;
                }
            } else {
                dentroDesde = -1;
                giro = limitar(Math.abs(erro) * GANHO_MIRA,
                    GIRO_MINIMO, GIRO_MAXIMO);
                if (erro > 0) giro = -giro;
            }
            esquerdo.setPower(giro);
            direito.setPower(-giro);
            lancador.atualizar(manterLancador);
            telemetry.addData("Mira webcam", decimal(erro) + " graus / " +
                alvo.metadata.name);
            telemetry.update();
            idle();
            estado = "LIMITE DE TEMPO";
        }
        esquerdo.setPower(0);
        direito.setPower(0);
        return estado;
    }

    private static AprilTagClusterDetection encontrarAlvo(AprilTagProcessor tags,
                                                            String alianca) {
        if (tags == null) return null;
        AprilTagClusterDetection melhor = null;
        for (AprilTagDetection deteccao : tags.getDetections()) {
            if (!(deteccao instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection celula = (AprilTagClusterDetection) deteccao;
            if (celula.metadata == null || celula.ftcPose == null) continue;
            if (!celula.metadata.name.toUpperCase().startsWith(
                    alianca.substring(0, 1))) continue;
            if (Math.abs(celula.ftcPose.roll) >= 90.0) continue;
            if (melhor == null || Math.abs(celula.ftcPose.bearing) <
                    Math.abs(melhor.ftcPose.bearing)) melhor = celula;
        }
        return melhor;
    }

    private static double correcao(double erro) {
        if (Math.abs(erro) <= FOLGA_TICKS) return 0;
        return limitar(Math.copySign(Math.abs(erro) - FOLGA_TICKS, erro) *
            GANHO_TICK, -CORRECAO_MAXIMA, CORRECAO_MAXIMA);
    }

    private static double misturar(double a, double b, double fracao) {
        return a + (b - a) * fracao;
    }

    private static double decimal(double valor) {
        return Math.round(valor * 10.0) / 10.0;
    }

    private static double limitar(double valor, double minimo, double maximo) {
        return Math.max(minimo, Math.min(maximo, valor));
    }
}

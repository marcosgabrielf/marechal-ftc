package org.firstinspires.ftc.teamcode;

import android.content.Context;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Formato v2: tempo, rodas, encoders e mecanismos. */
final class MarechalGravacao {
    static final double DURACAO_MAXIMA = 25.0;
    static final int MAX_SLOTS = 10;
    // O slot 1 usa o nome antigo para preservar o percurso ja gravado.
    private static final String ARQUIVO = "marechal_trajeto_gravado_v2.bin";
    private static final int ASSINATURA = 0x4D475232;
    private static final int MAX_AMOSTRAS = 650;

    static final class Amostra {
        final double tempo, esquerda, direita, coletor, pux;
        final int ticksEsquerdo, ticksDireito, alvoLancador;
        final boolean lancadorAtivo;

        Amostra(double tempo, double esquerda, double direita,
                int ticksEsquerdo, int ticksDireito, boolean lancadorAtivo,
                int alvoLancador, double coletor, double pux) {
            this.tempo = tempo;
            this.esquerda = esquerda;
            this.direita = direita;
            this.ticksEsquerdo = ticksEsquerdo;
            this.ticksDireito = ticksDireito;
            this.lancadorAtivo = lancadorAtivo;
            this.alvoLancador = alvoLancador;
            this.coletor = coletor;
            this.pux = pux;
        }
    }

    static final class Trajeto {
        final List<Amostra> amostras;
        final double duracao;
        final int polaridadeEsquerda, polaridadeDireita;

        Trajeto(List<Amostra> amostras, double duracao) {
            this.amostras = amostras;
            this.duracao = duracao;
            this.polaridadeEsquerda = polaridade(amostras, true);
            this.polaridadeDireita = polaridade(amostras, false);
        }

        boolean temEncoders() {
            return polaridadeEsquerda != 0 && polaridadeDireita != 0;
        }
    }

    private static int polaridade(List<Amostra> amostras, boolean esquerdo) {
        double correlacao = 0;
        int movimento = 0;
        for (int i = 1; i < amostras.size(); i++) {
            Amostra a = amostras.get(i - 1), b = amostras.get(i);
            int delta = esquerdo ? b.ticksEsquerdo - a.ticksEsquerdo :
                b.ticksDireito - a.ticksDireito;
            double potencia = esquerdo ? a.esquerda : a.direita;
            movimento += Math.abs(delta);
            if (Math.abs(potencia) > 0.08) correlacao += delta * potencia;
        }
        return movimento >= 30 && Math.abs(correlacao) >= 2 ?
            (correlacao > 0 ? 1 : -1) : 0;
    }

    static boolean existe(Context contexto, int slot) {
        return contexto.getFileStreamPath(nomeArquivo(slot)).exists();
    }

    private static String nomeArquivo(int slot) {
        if (slot < 1 || slot > MAX_SLOTS)
            throw new IllegalArgumentException("Slot fora de 1 a " + MAX_SLOTS);
        return slot == 1 ? ARQUIVO :
            "marechal_trajeto_gravado_v2_slot_" + slot + ".bin";
    }

    static void salvar(Context contexto, Trajeto trajeto, int slot) throws IOException {
        validar(trajeto);
        DataOutputStream saida = new DataOutputStream(
            contexto.openFileOutput(nomeArquivo(slot), Context.MODE_PRIVATE));
        try {
            saida.writeInt(ASSINATURA);
            saida.writeDouble(trajeto.duracao);
            saida.writeInt(trajeto.amostras.size());
            for (Amostra a : trajeto.amostras) {
                saida.writeDouble(a.tempo);
                saida.writeDouble(a.esquerda);
                saida.writeDouble(a.direita);
                saida.writeInt(a.ticksEsquerdo);
                saida.writeInt(a.ticksDireito);
                saida.writeBoolean(a.lancadorAtivo);
                saida.writeInt(a.alvoLancador);
                saida.writeDouble(a.coletor);
                saida.writeDouble(a.pux);
            }
        } finally {
            saida.close();
        }
    }

    static Trajeto carregar(Context contexto, int slot) throws IOException {
        DataInputStream entrada;
        try {
            entrada = new DataInputStream(contexto.openFileInput(nomeArquivo(slot)));
        } catch (FileNotFoundException erro) {
            throw new IOException("Slot " + slot + " vazio. Grave com X.");
        }
        try {
            if (entrada.readInt() != ASSINATURA)
                throw new IOException("Formato de gravacao invalido.");
            double duracao = entrada.readDouble();
            int quantidade = entrada.readInt();
            if (quantidade < 2 || quantidade > MAX_AMOSTRAS)
                throw new IOException("Quantidade de amostras invalida.");
            List<Amostra> amostras = new ArrayList<Amostra>(quantidade);
            for (int i = 0; i < quantidade; i++) {
                amostras.add(new Amostra(entrada.readDouble(), entrada.readDouble(),
                    entrada.readDouble(), entrada.readInt(), entrada.readInt(),
                    entrada.readBoolean(), entrada.readInt(), entrada.readDouble(),
                    entrada.readDouble()));
            }
            Trajeto trajeto = new Trajeto(amostras, duracao);
            validar(trajeto);
            return trajeto;
        } finally {
            entrada.close();
        }
    }

    private static void validar(Trajeto trajeto) throws IOException {
        if (!Double.isFinite(trajeto.duracao) || trajeto.duracao <= 0 ||
                trajeto.duracao > DURACAO_MAXIMA || trajeto.amostras.size() < 2 ||
                trajeto.amostras.size() > MAX_AMOSTRAS)
            throw new IOException("Duracao ou amostras invalidas.");
        double anterior = -1;
        for (Amostra a : trajeto.amostras) {
            if (!Double.isFinite(a.tempo) || a.tempo < anterior ||
                    a.tempo > trajeto.duracao + 0.001 ||
                    !potenciaValida(a.esquerda) || !potenciaValida(a.direita) ||
                    !potenciaValida(a.coletor) || !potenciaValida(a.pux) ||
                    a.alvoLancador < 700 || a.alvoLancador > 2200)
                throw new IOException("Amostra invalida.");
            anterior = a.tempo;
        }
    }

    private static boolean potenciaValida(double valor) {
        return Double.isFinite(valor) && Math.abs(valor) <= 1.001;
    }
}

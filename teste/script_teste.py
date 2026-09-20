#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teste MANUAL do Detector de Anomalias Acusticas (ESP32 + INMP441).

Fale perto do microfone quando o script pedir. Para cada tentativa o script
observa a serial do ESP32 e verifica se o LED foi acionado ("ACAO EXECUTADA").
So a classe "acende" DEVE acionar o LED; as demais (apaga, outra, ruido...) sao
negativas: se o LED acender, e falso positivo. Gera matriz de confusao, metricas
e latencia por etapa.

    pip install pyserial
    python -m serial.tools.list_ports          # descobrir a porta
    python test_detector_manual.py --port /dev/ttyUSB0
    python test_detector_manual.py --port /dev/ttyUSB0 --counts acende:20,apaga:20,outra:10,ruido:10

Saidas em resultados_teste/<data-hora>/: resultados.csv, resumo.md, serial_log.txt
"""
import argparse, csv, math, random, re, sys, threading, time
from collections import Counter
from datetime import datetime
from pathlib import Path

# Linha do firmware (Task 3), por exemplo:
# [Task3] slice=acende  (0.93) | RMS=4100.0 centroid~1500Hz | q1=0.2 T2=0.60 q2=0.1 DSP=22.3 NN=1.2 T3=25.0 | E2E=26.1ms  <-- ACAO EXECUTADA (alerta=27.3ms)
N = r"(-?[\d.]+)"
SLICE_RE = re.compile(rf"\[Task3\]\s+slice=(\S+)\s*\({N}\).*?q1={N}\s+T2={N}\s+q2={N}\s+"
                      rf"DSP={N}\s+NN={N}\s+T3={N}\s*\|\s*E2E={N}ms(.*)$")
ALERT_RE = re.compile(r"ACAO EXECUTADA\s*\(alerta=(-?[\d.]+)ms\)")
KEYS = ["conf", "q1", "t2", "q2", "dsp", "nn", "t3", "e2e"]
STAGES = [("q1", "Fila T1->T2"), ("t2", "Task 2 (RMS + ZCR)"), ("q2", "Fila T2->T3"),
          ("dsp", "Task 3 - DSP (MFCC)"), ("nn", "Task 3 - Rede neural"),
          ("t3", "Task 3 - total"),
          ("e2e", "Ponta-a-ponta (buffer cheio -> fim da classificacao)")]
ACOES = {"acende": "FALE 'acende'", "apaga": "FALE 'apaga'",
         "outra": "FALE OUTRA PALAVRA (ex.: 'oi', 'teste', 'bom dia')",
         "ruido": "FACA UM BARULHO (bata palma, bata na mesa) ou FIQUE EM SILENCIO"}


def parse(text, t=0.0):
    """Linha '[Task3] slice=...' -> dict (ou None). 'alert' = latencia buffer->LED, se houve acao."""
    m = SLICE_RE.search(text)
    if not m:
        return None
    g = m.groups()
    ev = {"t": t, "label": g[0], **dict(zip(KEYS, map(float, g[1:9])))}
    a = ALERT_RE.search(g[9])
    ev["alert"] = float(a.group(1)) if a else None
    return ev


def start_reader(port, baud, log_path):
    """Thread que le a serial, salva o log e acumula os slices. Devolve (slices, ready, erros)."""
    import serial
    ser = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
    slices, ready, err, log = [], threading.Event(), [], open(log_path, "w", encoding="utf-8")

    def loop():
        while True:
            try:
                raw = ser.readline()
            except Exception as e:
                err.append(e)
                return
            if not raw:
                continue
            t, text = time.perf_counter(), raw.decode("utf-8", "replace").rstrip("\r\n")
            log.write(f"{datetime.now().isoformat(timespec='milliseconds')} {text}\n")
            log.flush()
            if "Escutando" in text:
                ready.set()
            ev = parse(text, t)
            if ev:
                slices.append(ev)

    threading.Thread(target=loop, daemon=True).start()
    return slices, ready, err


def run_trial(i, total, cls, slices, a):
    """Instrui o usuario, observa a serial e devolve a linha do CSV."""
    print(f"[{i:>3}/{total}] Prepare-se: {ACOES.get(cls.lower(), f'FACA: {cls}')}", flush=True)
    for c in range(a.countdown, 0, -1):
        print(f"        {c}...", flush=True)
        time.sleep(1)
    print("        AGORA!", flush=True)
    t0 = time.perf_counter()
    time.sleep(a.speak_time + a.window)  # tempo de falar + espera do pipeline (slices de 250 ms)
    win = [s for s in slices if t0 <= s["t"] <= time.perf_counter()]
    alertas = [s for s in win if s["alert"] is not None]
    positivo = cls.lower() == "acende"
    res = ("TP" if alertas else "FN") if positivo else ("FP" if alertas else "TN")
    conf = max([s["conf"] for s in win if s["label"] == "acende"], default=0.0)
    print(f"        -> {res}  slices={len(win)} max(acende)={conf:.2f}"
          + (f"  alerta_fw={alertas[0]['alert']:.1f}ms" if alertas else ""), flush=True)
    time.sleep(a.gap)
    return {
        "trial": i, "classe": cls, "esperado_acionar": int(positivo),
        "acionou": int(bool(alertas)), "resultado": res, "n_alertas": len(alertas),
        "n_slices": len(win), "max_conf_acende": round(conf, 3),
        "labels_vistas": ";".join(f"{k}:{v}" for k, v in sorted(Counter(s["label"] for s in win).items())),
        "lat_alerta_firmware_ms": alertas[0]["alert"] if alertas else "",
    }


def latency_rows(slices):
    """Linhas da tabela de latencia: n / min / media / p50 / p95 / max (ms)."""
    fontes = [(nome, [s[k] for s in slices]) for k, nome in STAGES]
    fontes.append(("Alerta: buffer cheio -> LED (firmware)",
                   [s["alert"] for s in slices if s["alert"] is not None]))
    out = []
    for nome, v in fontes:
        if not v:
            continue
        s, n = sorted(v), len(v)
        p = lambda q: s[min(n - 1, max(0, math.ceil(q / 100 * n) - 1))]
        out.append(f"| {nome} | {n} | {s[0]:.2f} | {sum(s) / n:.2f} | {p(50):.2f} | {p(95):.2f} | {s[-1]:.2f} |")
    return out


def make_report(rows, slices, a):
    c = Counter(r["resultado"] for r in rows)
    tp, fn, fp, tn = c["TP"], c["FN"], c["FP"], c["TN"]
    pct = lambda x, y: f"{100 * x / y:.1f} %" if y else "n/d"
    f1 = f"{2 * tp / (2 * tp + fp + fn):.3f}" if (2 * tp + fp + fn) else "n/d"
    L = ["# Resultado do teste manual - Detector de Anomalias Acusticas", "",
         f"- **Data:** {datetime.now():%d/%m/%Y %H:%M}", f"- **Porta:** {a.port}",
         f"- **Tentativas:** {len(rows)}",
         f"- **Protocolo:** contagem de {a.countdown} s, {a.speak_time} s para falar, "
         f"{a.window} s de observacao, ordem {'embaralhada' if a.shuffle else 'fixa'}", "",
         "## Matriz de confusao (acao do LED)", "",
         "| | LED acionou | LED nao acionou |", "|---|---:|---:|",
         f"| **Deveria acionar** ('acende') | TP = {tp} | FN = {fn} |",
         f"| **Nao deveria acionar** | FP = {fp} | TN = {tn} |", "",
         "| Metrica | Valor |", "|---|---:|", f"| Tentativas | {len(rows)} |",
         f"| Acuracia | {pct(tp + tn, len(rows))} |", f"| Precisao | {pct(tp, tp + fp)} |",
         f"| Recall (sensibilidade) | {pct(tp, tp + fn)} |", f"| F1 | {f1} |",
         f"| Taxa de falso positivo | {pct(fp, fp + tn)} |", "",
         "## Por classe", "", "| Classe | Tentativas | LED acionou | Taxa de acionamento |",
         "|---|---:|---:|---:|"]
    for cls in sorted({r["classe"] for r in rows}):
        rs = [r for r in rows if r["classe"] == cls]
        ac = sum(r["acionou"] for r in rs)
        L.append(f"| {cls} | {len(rs)} | {ac} | {pct(ac, len(rs))} |")
    L += ["", "## Latencia por etapa (slices recebidos durante o teste)", "",
          "| Etapa | n | min (ms) | media (ms) | p50 (ms) | p95 (ms) | max (ms) |",
          "|---|---:|---:|---:|---:|---:|---:|"] + latency_rows(slices)
    L += ["", "_Slices em silencio (abaixo do gate de RMS) nao chegam a Task 3 e ficam fora "
              "destas estatisticas._", ""]
    return "\n".join(L)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add = p.add_argument
    add("--port", help="porta serial (COM5, /dev/ttyUSB0...)")
    add("--baud", type=int, default=115200)
    add("--counts", default="acende:15,apaga:15,outra:10,ruido:5",
        help="classe:quantidade,... (so 'acende' deve acionar o LED)")
    add("--no-shuffle", dest="shuffle", action="store_false", help="mantem a ordem das classes")
    add("--seed", type=int, default=42)
    add("--countdown", type=int, default=3, help="contagem regressiva (s)")
    add("--speak-time", type=float, default=1.5, help="tempo para falar (s)")
    add("--window", type=float, default=2.0, help="observacao apos a fala (s)")
    add("--gap", type=float, default=1.0, help="pausa entre tentativas (janela+pausa > tempo do LED aceso)")
    add("--out", default="resultados_teste")
    add("--yes", action="store_true", help="nao pede ENTER para iniciar")
    a = p.parse_args()
    if not a.port:
        sys.exit("Informe --port (descubra com: python -m serial.tools.list_ports)")

    trials = []
    for part in a.counts.split(","):
        nome, _, k = part.partition(":")
        trials += [nome.strip()] * int(k or 1)
    if a.shuffle:
        random.Random(a.seed).shuffle(trials)

    out = Path(a.out) / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    slices, ready, err = start_reader(a.port, a.baud, out / "serial_log.txt")
    print(f"Porta {a.port} aberta. Aguardando o firmware iniciar...")
    if not ready.wait(10):
        print("AVISO: nao vi 'Escutando'; o ESP32 pode ja estar rodando. Prosseguindo.")
    print(f"{len(trials)} tentativas.")
    if not a.yes:
        input("Posicione-se perto do microfone e pressione ENTER... ")

    rows, t_run = [], time.perf_counter()
    try:
        for i, cls in enumerate(trials, 1):
            if err:
                print(f"ERRO na serial: {err[0]}")
                break
            rows.append(run_trial(i, len(trials), cls, slices, a))
    except KeyboardInterrupt:
        print("\nInterrompido - gerando relatorio parcial.")

    if rows:
        with open(out / "resultados.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    report = make_report(rows, [s for s in slices if s["t"] >= t_run], a)
    (out / "resumo.md").write_text(report, encoding="utf-8")
    print("\n" + report + f"\nArquivos salvos em: {out}")


if __name__ == "__main__":
    main()

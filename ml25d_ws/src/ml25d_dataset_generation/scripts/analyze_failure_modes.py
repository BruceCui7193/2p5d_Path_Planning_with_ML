#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
from collections import Counter, defaultdict

import h5py
import numpy as np


def _decode_text(x):
    return x.decode("utf-8") if isinstance(x, (bytes, bytearray)) else str(x)


def _load(pattern: str):
    files = sorted(glob.glob(pattern))
    if not files:
        raise SystemExit(f"no files matched: {pattern}")

    ys = []
    bands = []
    metas = []
    for fp in files:
        with h5py.File(fp, "r") as f:
            ys.append(f["y"][:])
            bands.extend([_decode_text(b) for b in f["band"][:]])
            metas.extend([json.loads(_decode_text(m)) for m in f["metadata_json"][:]])

    y = np.concatenate(ys, axis=0)
    return y, bands, metas


def _summary(y: np.ndarray, bands, metas):
    roll = y[:, 1]
    pitch = y[:, 2]
    slip = y[:, 3]
    lift = y[:, 4]
    bottom = y[:, 5]
    stuck = y[:, 6]

    trigger = {
        "roll>0.9": (roll > 0.9),
        "pitch>0.9": (pitch > 0.9),
        "slip>0.8": (slip > 0.8),
        "lift>0.25": (lift > 0.25),
        "bottom>0.2": (bottom > 0.2),
        "stuck>=1": (stuck >= 1.0),
    }

    by_action = defaultdict(lambda: {"n": 0, "fail": 0, "slip": 0, "bottom": 0, "stuck": 0})
    by_model = defaultdict(lambda: {"n": 0, "fail": 0, "slip": 0, "bottom": 0, "stuck": 0})

    for i, m in enumerate(metas):
        a = m.get("action_id", "?")
        mm = m.get("motion_model", "?")
        for table in (by_action[a], by_model[mm]):
            table["n"] += 1
            table["fail"] += int(y[i, 0] >= 0.5)
            table["slip"] += int(trigger["slip>0.8"][i])
            table["bottom"] += int(trigger["bottom>0.2"][i])
            table["stuck"] += int(trigger["stuck>=1"][i])

    def _norm_table(tab):
        out = {}
        for k, v in sorted(tab.items()):
            n = max(v["n"], 1)
            out[k] = {
                "n": v["n"],
                "fail_rate": v["fail"] / n,
                "slip_trigger_rate": v["slip"] / n,
                "bottom_trigger_rate": v["bottom"] / n,
                "stuck_trigger_rate": v["stuck"] / n,
            }
        return out

    summary = {
        "num_samples": int(y.shape[0]),
        "band_counts": dict(Counter(bands)),
        "label_mean": {
            "y_fail": float(y[:, 0].mean()),
            "q_roll": float(roll.mean()),
            "q_pitch": float(pitch.mean()),
            "q_slip": float(slip.mean()),
            "q_lift": float(lift.mean()),
            "p_bottom": float(bottom.mean()),
            "p_stuck": float(stuck.mean()),
        },
        "trigger_counts": {k: int(v.sum()) for k, v in trigger.items()},
        "by_action": _norm_table(by_action),
        "by_motion_model": _norm_table(by_model),
    }

    # Heuristic diagnosis for code-vs-config contribution
    code_score = summary["trigger_counts"]["slip>0.8"] + summary["trigger_counts"]["stuck>=1"]
    config_score = summary["trigger_counts"]["bottom>0.2"] + summary["trigger_counts"]["roll>0.9"]
    if code_score > 1.25 * config_score:
        root = "motion_control_dominant"
    elif config_score > 1.25 * code_score:
        root = "terrain_or_threshold_dominant"
    else:
        root = "mixed"
    summary["diagnosis"] = {
        "root_cause_type": root,
        "code_score": int(code_score),
        "config_score": int(config_score),
    }

    return summary


def _plot(summary: dict, output_dir: Path):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    output_dir.mkdir(parents=True, exist_ok=True)

    trig = summary["trigger_counts"]
    keys = list(trig.keys())
    vals = [trig[k] for k in keys]

    plt.figure(figsize=(10, 4))
    plt.bar(keys, vals)
    plt.title("Failure Trigger Counts")
    plt.xticks(rotation=20, ha="right")
    plt.tight_layout()
    plt.savefig(output_dir / "failure_trigger_counts.png", dpi=150)
    plt.close()

    by_action = summary["by_action"]
    aks = list(by_action.keys())
    fail = [by_action[k]["fail_rate"] for k in aks]
    slip = [by_action[k]["slip_trigger_rate"] for k in aks]
    bottom = [by_action[k]["bottom_trigger_rate"] for k in aks]

    x = np.arange(len(aks))
    w = 0.25
    plt.figure(figsize=(10, 4))
    plt.bar(x - w, fail, width=w, label="fail")
    plt.bar(x, slip, width=w, label="slip")
    plt.bar(x + w, bottom, width=w, label="bottom")
    plt.xticks(x, aks)
    plt.ylim(0.0, 1.0)
    plt.title("Rates by Action")
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "rates_by_action.png", dpi=150)
    plt.close()

    return True


def main() -> int:
    p = argparse.ArgumentParser(description="Analyze dataset failure modes and produce visual diagnostics")
    p.add_argument("--pattern", required=True, help="Glob pattern, e.g. data/xxx/samples_batch_*.h5")
    p.add_argument("--output-dir", default="data/analysis", help="Output directory for json/png")
    args = p.parse_args()

    y, bands, metas = _load(args.pattern)
    summary = _summary(y, bands, metas)

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    summary_path = out / "failure_analysis_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=True), encoding="utf-8")

    plotted = _plot(summary, out)
    print(json.dumps({"summary": str(summary_path), "plotted": plotted}, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

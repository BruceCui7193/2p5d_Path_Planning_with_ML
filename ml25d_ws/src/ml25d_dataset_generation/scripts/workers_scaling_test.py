#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run workers scaling benchmark for parallel dataset generation")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--workers", type=str, default="4,6,8,10,12", help="Comma-separated worker counts")
    parser.add_argument("--num-samples", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260427)
    parser.add_argument("--base-domain-start", type=int, default=150)
    parser.add_argument("--backend", type=str, default="ros_gz", choices=["ros_gz", "mock", "surrogate"])
    parser.add_argument(
        "--generator-script",
        type=Path,
        default=Path("src/ml25d_dataset_generation/scripts/generate_pilot_parallel.py"),
    )
    return parser.parse_args()


def _parse_workers(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        p = part.strip()
        if not p:
            continue
        out.append(int(p))
    return out


def _parse_elapsed_to_sec(text: str) -> float:
    # Accept h:mm:ss or m:ss
    text = text.strip()
    parts = text.split(":")
    if len(parts) == 3:
        h, m, s = parts
        return float(h) * 3600.0 + float(m) * 60.0 + float(s)
    if len(parts) == 2:
        m, s = parts
        return float(m) * 60.0 + float(s)
    return float("nan")


def _extract_time_metrics(stderr_text: str) -> dict[str, float]:
    out = {
        "wall_sec": float("nan"),
        "cpu_percent": float("nan"),
        "max_rss_kb": float("nan"),
    }
    m = re.search(r"Elapsed \(wall clock\) time \(h:mm:ss or m:ss\):\s*([0-9:.]+)", stderr_text)
    if m:
        out["wall_sec"] = _parse_elapsed_to_sec(m.group(1))
    m = re.search(r"Percent of CPU this job got:\s*([0-9.]+)%", stderr_text)
    if m:
        out["cpu_percent"] = float(m.group(1))
    m = re.search(r"Maximum resident set size \(kbytes\):\s*([0-9]+)", stderr_text)
    if m:
        out["max_rss_kb"] = float(m.group(1))
    return out


def _count_conflict_strings(path: Path) -> int:
    if not path.exists():
        return 0
    total = 0
    for f in path.rglob("*"):
        if not f.is_file():
            continue
        if f.stat().st_size == 0:
            continue
        if f.suffix not in {".log", ".txt", ".jsonl", ".json", ".md"}:
            continue
        try:
            text = f.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            continue
        total += text.lower().count("already exists")
        total += text.lower().count("entity_name_conflict")
    return total


def _fmt(x: Any) -> str:
    if isinstance(x, float):
        if x != x:
            return "NaN"
        return f"{x:.4f}"
    return str(x)


def _table(rows: list[dict[str, Any]], headers: list[str]) -> str:
    lines = []
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        lines.append("| " + " | ".join(_fmt(row.get(h, "")) for h in headers) + " |")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    workers_list = _parse_workers(args.workers)
    if not workers_list:
        raise ValueError("workers list is empty")

    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    gen_script = args.generator_script.resolve()
    if not gen_script.exists():
        raise FileNotFoundError(f"generator script not found: {gen_script}")

    rows: list[dict[str, Any]] = []
    for idx, n_workers in enumerate(workers_list):
        run_dir = output_root / f"workers_{n_workers}"
        if run_dir.exists():
            shutil.rmtree(run_dir)
        run_dir.mkdir(parents=True, exist_ok=True)
        dataset_dir = run_dir / "dataset"
        stdout_log = run_dir / "stdout.log"
        stderr_log = run_dir / "stderr.log"

        base_domain = int(args.base_domain_start) + idx * 12
        seed = int(args.seed) + idx * 17

        cmd = [
            "/usr/bin/time",
            "-v",
            sys.executable,
            str(gen_script),
            "--output-dir",
            str(dataset_dir),
            "--num-samples",
            str(args.num_samples),
            "--seed",
            str(seed),
            "--backend",
            str(args.backend),
            "--num-workers",
            str(n_workers),
            "--base-domain",
            str(base_domain),
            "--flush-batch-size",
            str(args.num_samples),
        ]

        print(
            f"[scaling] start workers={n_workers} num_samples={args.num_samples} "
            f"base_domain={base_domain} seed={seed}",
            flush=True,
        )
        proc = subprocess.run(cmd, capture_output=True, text=True)
        stdout_log.write_text(proc.stdout, encoding="utf-8")
        stderr_log.write_text(proc.stderr, encoding="utf-8")

        time_metrics = _extract_time_metrics(proc.stderr)
        manifest_path = dataset_dir / "dataset_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
        invalid_reasons = manifest.get("counts", {}).get("invalid_reason", {})
        conflict_reason_count = int(invalid_reasons.get("entity_name_conflict", 0))
        conflict_string_hits = _count_conflict_strings(dataset_dir)
        accepted = int(manifest.get("dataset", {}).get("num_samples_accepted", 0))
        invalid_rate = float(manifest.get("dataset", {}).get("invalid_sample_rate", float("nan")))
        wall_sec = float(time_metrics.get("wall_sec", float("nan")))
        samples_per_min = float(accepted / (wall_sec / 60.0)) if wall_sec == wall_sec and wall_sec > 0 else float("nan")

        row = {
            "workers": int(n_workers),
            "return_code": int(proc.returncode),
            "accepted_samples": accepted,
            "wall_sec": wall_sec,
            "samples_per_min": samples_per_min,
            "invalid_rate": invalid_rate,
            "invalid_attempts": int(manifest.get("dataset", {}).get("invalid_attempts", 0)),
            "entity_conflict_invalid": conflict_reason_count,
            "already_exists_hits": int(conflict_string_hits),
            "cpu_percent": float(time_metrics.get("cpu_percent", float("nan"))),
            "max_rss_gb": float(time_metrics.get("max_rss_kb", float("nan")) / (1024.0 * 1024.0))
            if time_metrics.get("max_rss_kb", float("nan")) == time_metrics.get("max_rss_kb", float("nan"))
            else float("nan"),
            "run_dir": str(run_dir),
        }
        rows.append(row)
        print(
            f"[scaling] done workers={n_workers} rc={proc.returncode} "
            f"samples_per_min={samples_per_min:.2f} invalid_rate={invalid_rate:.4f}",
            flush=True,
        )

    stable = [
        r
        for r in rows
        if int(r["return_code"]) == 0
        and float(r["invalid_rate"]) <= 0.05
        and int(r["entity_conflict_invalid"]) == 0
        and int(r["already_exists_hits"]) == 0
    ]
    if stable:
        best = max(stable, key=lambda x: float(x["samples_per_min"]))
        selected_workers = int(best["workers"])
        selected_reason = "stable_and_fastest"
    else:
        selected_workers = int(max(rows, key=lambda x: float(x["samples_per_min"]))["workers"])
        selected_reason = "fallback_fastest_no_fully_stable_config"

    summary = {
        "num_samples_per_run": int(args.num_samples),
        "backend": args.backend,
        "rows": rows,
        "selected_workers": selected_workers,
        "selected_reason": selected_reason,
    }

    json_path = output_root / "workers_scaling_summary.json"
    md_path = output_root / "workers_scaling_summary.md"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    md_rows = []
    for r in rows:
        md_rows.append(
            {
                "workers": r["workers"],
                "samples_per_min": r["samples_per_min"],
                "wall_sec": r["wall_sec"],
                "invalid_rate": r["invalid_rate"],
                "invalid_attempts": r["invalid_attempts"],
                "entity_conflict_invalid": r["entity_conflict_invalid"],
                "already_exists_hits": r["already_exists_hits"],
                "cpu_percent": r["cpu_percent"],
                "max_rss_gb": r["max_rss_gb"],
                "return_code": r["return_code"],
            }
        )
    md = []
    md.append("## Workers Scaling Summary")
    md.append("")
    md.append(
        _table(
            md_rows,
            [
                "workers",
                "samples_per_min",
                "wall_sec",
                "invalid_rate",
                "invalid_attempts",
                "entity_conflict_invalid",
                "already_exists_hits",
                "cpu_percent",
                "max_rss_gb",
                "return_code",
            ],
        )
    )
    md.append("")
    md.append(f"selected_workers: `{selected_workers}`")
    md.append(f"selected_reason: `{selected_reason}`")
    md.append("")
    md.append(f"json_path: `{json_path}`")
    md.append(f"md_path: `{md_path}`")
    md_path.write_text("\n".join(md) + "\n", encoding="utf-8")

    print(json.dumps({"json": str(json_path), "md": str(md_path), "selected_workers": selected_workers}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

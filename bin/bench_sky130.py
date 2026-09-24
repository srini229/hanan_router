#!/usr/bin/env python3
"""Run hanan_router on every sky130-benchmarks circuit and tabulate the result.

Uses the router-ready inputs the benchmark harness leaves in
<bench>/bench/harness/runs/<circuit>/ (placement.json, combined.lef, and
route_ndr.json where the circuit has symmetric nets) and the sky130 layers.json
from the same tree. Writes a Markdown table (and, with --html, an HTML one)
with, per circuit: nets, unrouted, router-caused DRC violations, total
wirelength, wall-clock seconds -- and, with --baseline, the same for a second
binary plus the deltas.

    bin/bench_sky130.py                         # this repo's ./hanan_router
    bin/bench_sky130.py --rsmt                  # with auto Steiner corridors
    bin/bench_sky130.py --baseline /old/hanan_router --rsmt --out cmp.md --html cmp.html
    bin/bench_sky130.py --only mfb_biquad,ring_vco --extra "-reorder 10"

The benchmark tree is taken from --bench, else $SKY130_BENCH, else the
default below. Per-circuit logs go under --workdir (default: a temp dir).
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
DEFAULT_BENCH = "/data1/scratch/sky130-benchmarks"


def find_circuits(bench, only):
    runs = os.path.join(bench, "bench", "harness", "runs")
    out = []
    for name in sorted(os.listdir(runs)):
        d = os.path.join(runs, name)
        if only and name not in only:
            continue
        if os.path.isfile(os.path.join(d, "placement.json")) and os.path.isfile(os.path.join(d, "combined.lef")):
            out.append((name, d))
    return out


def run_one(router, layers, name, d, workdir, rsmt, extra, timeout):
    out = os.path.join(workdir, name)
    os.makedirs(out, exist_ok=True)
    cmd = [router, "-d", layers, "-p", os.path.join(d, "placement.json"),
           "-l", os.path.join(d, "combined.lef"), "-uu", "1000", "-reorder", "30",
           "-o", out + "/"]
    ndr = os.path.join(d, "route_ndr.json")
    if os.path.isfile(ndr):
        cmd += ["-ndr", ndr]
    if rsmt:
        cmd.append("-rsmt")
    cmd += extra
    t0 = time.time()
    try:
        r = subprocess.run(cmd, cwd=out, capture_output=True, text=True, timeout=timeout)
        rc = r.returncode
    except subprocess.TimeoutExpired:
        rc = "timeout"
    secs = time.time() - t0
    log_path = os.path.join(out, "route.log")   # the router writes this itself
    log = open(log_path, errors="replace").read() if os.path.isfile(log_path) else ""
    def grab(rx, conv=int):
        m = re.search(rx, log)
        return conv(m.group(1)) if m else None
    nets = grab(r"ROUTE_SUMMARY module=\S+ nets=(\d+)")
    vias = 0
    for fn in os.listdir(out):
        if fn.endswith(".def"):
            vias += len(re.findall(r"^\s*\+ RECT V\d", open(os.path.join(out, fn), errors="replace").read(), re.M))
    return {
        "circuit": name, "rc": rc, "secs": secs,
        "nets": nets,
        "unrouted": grab(r"ROUTE_SUMMARY module=\S+ nets=\d+ unrouted=(\d+)"),
        "drc": grab(r"DRC_SUMMARY router-caused spacing violations = (\d+)"),
        "wirelength": grab(r"WIRELENGTH TOTAL \S+ : (\d+)"),
        "vias": vias,
        "log": log_path,
    }


def fmt(v, nd=0):
    if v is None:
        return "-"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def pct(a, b):
    if a in (None, 0) or b is None:
        return "-"
    return f"{(b - a) / a * 100:+.1f}%"


def md_table(rows, base):
    if base:
        head = ["circuit", "nets", "unrouted", "DRC", "wirelength", "vias", "s", "base wl", "wl Δ", "base vias", "base s", "s Δ"]
        lines = ["| " + " | ".join(head) + " |", "|" + "|".join(["---"] * len(head)) + "|"]
        for r, b in zip(rows, base):
            lines.append("| " + " | ".join([
                r["circuit"], fmt(r["nets"]), fmt(r["unrouted"]), fmt(r["drc"]), fmt(r["wirelength"]),
                fmt(r["vias"]), fmt(r["secs"], 2), fmt(b["wirelength"]), pct(b["wirelength"], r["wirelength"]),
                fmt(b["vias"]), fmt(b["secs"], 2), pct(b["secs"], r["secs"])]) + " |")
        tw = sum(r["secs"] for r in rows); tb = sum(b["secs"] for b in base)
        lines.append(f"| **total** | | {sum(r['unrouted'] or 0 for r in rows)} | {sum(r['drc'] or 0 for r in rows)} | "
                     f"{sum(r['wirelength'] or 0 for r in rows)} | {sum(r['vias'] for r in rows)} | {tw:.2f} | {sum(b['wirelength'] or 0 for b in base)} | "
                     f"{pct(sum(b['wirelength'] or 0 for b in base), sum(r['wirelength'] or 0 for r in rows))} | {sum(b['vias'] for b in base)} | {tb:.2f} | {pct(tb, tw)} |")
    else:
        head = ["circuit", "nets", "unrouted", "DRC", "wirelength", "vias", "s"]
        lines = ["| " + " | ".join(head) + " |", "|" + "|".join(["---"] * len(head)) + "|"]
        for r in rows:
            lines.append("| " + " | ".join([r["circuit"], fmt(r["nets"]), fmt(r["unrouted"]), fmt(r["drc"]),
                                            fmt(r["wirelength"]), fmt(r["vias"]), fmt(r["secs"], 2)]) + " |")
        lines.append(f"| **total** | | {sum(r['unrouted'] or 0 for r in rows)} | {sum(r['drc'] or 0 for r in rows)} | "
                     f"{sum(r['wirelength'] or 0 for r in rows)} | {sum(r['vias'] for r in rows)} | {sum(r['secs'] for r in rows):.2f} |")
    return "\n".join(lines)


def html_table(md):
    rows = [l for l in md.splitlines() if l.startswith("|") and not set(l) <= set("|-")]
    cells = [[c.strip().replace("**", "") for c in l.strip("|").split("|")] for l in rows]
    h = ["<table>", "<thead><tr>" + "".join(f"<th>{c}</th>" for c in cells[0]) + "</tr></thead>", "<tbody>"]
    for row in cells[1:]:
        h.append("<tr>" + "".join(f"<td>{c}</td>" for c in row) + "</tr>")
    h += ["</tbody>", "</table>"]
    return "\n".join(h)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", default=os.environ.get("SKY130_BENCH", DEFAULT_BENCH))
    ap.add_argument("--router", default=os.path.join(REPO, "hanan_router"))
    ap.add_argument("--baseline", help="a second router binary to run and compare against")
    ap.add_argument("--rsmt", action="store_true", help="pass -rsmt (auto Steiner corridors)")
    ap.add_argument("--extra", default="", help="extra router flags, quoted, e.g. \"-reorder 10 -v 2\"")
    ap.add_argument("--only", help="comma-separated circuit names")
    ap.add_argument("--timeout", type=int, default=900)
    ap.add_argument("--workdir", help="where per-circuit route.log/DEF go (default: temp dir)")
    ap.add_argument("--out", default="bench_sky130.md", help="Markdown table path")
    ap.add_argument("--html", help="also write an HTML table here")
    ap.add_argument("--layers", help="layer file (default: the benchmark's sky130.layers.json)")
    a = ap.parse_args()

    layers = a.layers or os.path.join(a.bench, "bench", "primitives", "lib", "sky130.layers.json")
    for p, what in ((a.router, "router"), (layers, "layers.json")):
        if not os.path.exists(p):
            sys.exit(f"{what} not found: {p}")
    circuits = find_circuits(a.bench, set(a.only.split(",")) if a.only else None)
    if not circuits:
        sys.exit(f"no circuits with placement.json + combined.lef under {a.bench}/bench/harness/runs")
    workdir = a.workdir or tempfile.mkdtemp(prefix="bench_sky130_")
    extra = a.extra.split()

    def sweep(router, tag):
        rows = []
        for name, d in circuits:
            r = run_one(router, layers, name, d, os.path.join(workdir, tag), a.rsmt, extra, a.timeout)
            rows.append(r)
            print(f"{tag:8s} {name:22s} rc={r['rc']!s:8s} nets={fmt(r['nets']):>3s} unrouted={fmt(r['unrouted']):>2s} "
                  f"drc={fmt(r['drc']):>2s} wl={fmt(r['wirelength']):>8s} vias={r['vias']:>4d} {r['secs']:7.2f}s", flush=True)
        return rows

    rows = sweep(a.router, "new")
    base = sweep(a.baseline, "base") if a.baseline else None

    title = f"# hanan_router on sky130-benchmarks{' (-rsmt)' if a.rsmt else ''}\n\n"
    meta = (f"router: `{a.router}`  \n" + (f"baseline: `{a.baseline}`  \n" if a.baseline else "") +
            f"flags: `-uu 1000 -reorder 30{' -rsmt' if a.rsmt else ''}{(' ' + a.extra) if a.extra else ''}`  \n"
            f"logs: `{workdir}`\n\n")
    md = title + meta + md_table(rows, base) + "\n"
    with open(a.out, "w") as f:
        f.write(md)
    if a.html:
        with open(a.html, "w") as f:
            f.write("<!doctype html><meta charset=utf-8><title>hanan_router sky130 benchmark</title>"
                    "<style>body{font:14px system-ui;margin:2em}table{border-collapse:collapse}"
                    "td,th{border:1px solid #ccc;padding:4px 10px;text-align:right}td:first-child,th:first-child{text-align:left}</style>"
                    + "<h1>" + title.strip("# \n") + "</h1><pre>" + meta.replace("  \n", "\n") + "</pre>" + html_table(md))
    print(f"\nwrote {a.out}" + (f" and {a.html}" if a.html else ""))
    print(md_table(rows, base))
    bad = [r["circuit"] for r in rows if r["rc"] != 0 or (r["unrouted"] or 0) or (r["drc"] or 0)]
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()

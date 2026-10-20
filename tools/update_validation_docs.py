#!/usr/bin/env python3
"""Rewrite the generated tables in the docs from the validation JSON.

    AVD_VALIDATION_JSON=build/validation.json ./build/tests/avd_tests validation
    # or: ./build/avdemo validate --json build/validation.json
    python3 tools/update_validation_docs.py build/validation.json [--check]

Every block between `<!-- validation:begin <spec> -->` and
`<!-- validation:end -->` in README.md, docs/fidelity-matrix.md and
docs/validation/*.md is regenerated; everything outside the markers is left
alone. `<spec>` is one of

    table aircraft=<id> [prefix=<row-id prefix>] [kinds=tolerance,range,info]
        the comparison table for that aircraft's rows (all, or the ids with
        the prefix), in the JSON's order
    summary aircraft=<id>
        one line: "N rows, M outside their band"
    tests [file=<test_*.cpp>]
        the number of TEST_CASE()s in tests/ (or in one file), counted from
        the sources so the docs' test counts cannot drift
    value id=<row id> [field=value|reference|error_pct|low|high] [fmt=<python format>]
        one number inline

With --check nothing is written; the exit status is 1 if any file would
change (CI runs it after a build and fails on stale docs). Standard library
only; the script is idempotent.
"""
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = ["README.md", "docs/fidelity-matrix.md", "docs/roadmap.md", "web/README.md"] + sorted(
    os.path.join("docs/validation", f) for f in os.listdir(os.path.join(ROOT, "docs/validation")) if f.endswith(".md")
)
BEGIN = re.compile(r"<!-- validation:begin (.*?) -->")
END = "<!-- validation:end -->"


def parse_spec(text):
    parts = text.split()
    kind = parts[0]
    args = {}
    for p in parts[1:]:
        k, _, v = p.partition("=")
        args[k] = v
    return kind, args


def fmt_num(v, unit=""):
    if v is None:
        return "—"
    av = abs(v)
    if v == int(v) and av < 1e15:
        s = f"{int(v):,d}"
    elif av >= 1000:
        s = f"{v:,.0f}"
    elif av >= 100:
        s = f"{v:.0f}"
    elif av >= 10:
        s = f"{v:.1f}"
    else:
        s = f"{v:.2f}"
    return s


def row_line(r):
    unit = r["unit"]
    value = f"{fmt_num(r['value'])} {unit}".strip()
    if r["kind"] == "tolerance":
        err = f"{r['error_pct']:+.1f} %"
        band = f"±{r['tolerance_pct']:.0f} %"
    elif r["kind"] == "range":
        err = "—"
        band = f"{fmt_num(r['low'])}–{fmt_num(r['high'])} {unit}".strip()
    else:
        err = "—"
        band = "—"
    status = "info" if r["kind"] == "info" else ("within" if r["pass"] else "**OUTSIDE**")
    published = r["published"] or "—"
    quantity = r["quantity"]
    if r.get("note"):
        quantity += f"<br><small>{r['note']}</small>"
    source = r.get("source") or "—"
    return f"| {quantity} | {published} | {value} | {err} | {band} | {status} | {source} |"


def gen_table(rows, args):
    aircraft = args.get("aircraft")
    prefix = args.get("prefix", "")
    kinds = set(args.get("kinds", "tolerance,range,info").split(","))
    sel = [r for r in rows if (not aircraft or r["aircraft"] == aircraft) and r["id"].startswith(prefix) and r["kind"] in kinds]
    out = ["| Quantity | Published | Simulation | Error | Band | Result | Source |", "|---|---|---|---|---|---|---|"]
    out += [row_line(r) for r in sel]
    return "\n".join(out)


def gen_summary(rows, args):
    aircraft = args.get("aircraft")
    sel = [r for r in rows if (not aircraft or r["aircraft"] == aircraft) and r["kind"] != "info"]
    outside = [r for r in sel if not r["pass"]]
    s = f"{len(sel)} compared rows, {len(outside)} outside their band"
    if outside:
        s += " (" + ", ".join(r["id"] for r in outside) + ")"
    return s + "."


def count_tests(args):
    tests_dir = os.path.join(ROOT, "tests")
    files = [args["file"]] if "file" in args else sorted(f for f in os.listdir(tests_dir) if f.startswith("test_") and f.endswith(".cpp"))
    n = 0
    for f in files:
        with open(os.path.join(tests_dir, f), encoding="utf-8") as fh:
            n += len(re.findall(r"^TEST_CASE\(", fh.read(), re.M))
    return str(n)


def gen_value(rows, args):
    for r in rows:
        if r["id"] == args["id"]:
            v = r[args.get("field", "value")]
            if "fmt" in args:
                return format(v, args["fmt"])
            return fmt_num(v)
    raise SystemExit(f"no row {args['id']} in the JSON")


def generate(kind, args, rows):
    if kind == "table":
        return gen_table(rows, args)
    if kind == "summary":
        return gen_summary(rows, args)
    if kind == "tests":
        return count_tests(args)
    if kind == "value":
        return gen_value(rows, args)
    raise SystemExit(f"unknown block kind '{kind}'")


def rewrite(text, rows):
    out = []
    pos = 0
    while True:
        m = BEGIN.search(text, pos)
        if not m:
            out.append(text[pos:])
            break
        end = text.find(END, m.end())
        if end < 0:
            raise SystemExit(f"unterminated validation block: {m.group(0)}")
        kind, args = parse_spec(m.group(1))
        body = generate(kind, args, rows)
        inline = kind in ("tests", "value")
        out.append(text[pos:m.end()])
        out.append(body if inline else "\n" + body + "\n")
        pos = end
    return "".join(out)


def main(argv):
    check = "--check" in argv
    paths = [a for a in argv[1:] if not a.startswith("--")]
    rows = []
    if paths:
        with open(paths[0], encoding="utf-8") as fh:
            data = json.load(fh)
        if data.get("format") != "avd-validation/1":
            raise SystemExit("not an avd-validation/1 JSON")
        if not data.get("complete") and "--allow-partial" not in argv:
            raise SystemExit("the JSON is incomplete (a filtered test run); run the whole validation group or pass --allow-partial")
        rows = data["rows"]
    stale = []
    for rel in DOCS:
        path = os.path.join(ROOT, rel)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as fh:
            old = fh.read()
        if not BEGIN.search(old):
            continue
        if not rows and re.search(r"<!-- validation:begin (table|summary|value)", old):
            raise SystemExit(f"{rel} has table/summary/value blocks: pass the validation JSON")
        new = rewrite(old, rows)
        if new != old:
            stale.append(rel)
            if not check:
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(new)
    if check:
        if stale:
            print("stale generated tables in: " + ", ".join(stale) + " (run tools/update_validation_docs.py)")
            return 1
        print("generated tables are current")
        return 0
    print(("updated: " + ", ".join(stale)) if stale else "nothing to update")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

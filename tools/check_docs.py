#!/usr/bin/env python3
"""Check that the docs still point at things that exist.

For CLAUDE.md and docs/agents/*.md:
  * every relative markdown link resolves to a file;
  * every `kSomething` constant mentioned in backticks is still defined or used under src/ or tests/;
  * every point attribute named in src/sync/Box3DAttribNames.h is mentioned in backticks in README.md
    (the README's attribute table is prose; this is what keeps it in step with the code).

Docs name constants instead of copying their values, so a rename in code shows up here.
Usage: python tools/check_docs.py   (exit code 1 on any problem)
"""
import re
import sys
from pathlib import Path

root = Path(__file__).resolve().parent.parent
docs = [root / "CLAUDE.md", *sorted((root / "docs" / "agents").glob("*.md"))]

code = "\n".join(
    p.read_text(encoding="utf-8", errors="replace")
    for d in ("src", "tests")
    for p in (root / d).rglob("*")
    if p.suffix in (".h", ".cpp")
)
known = set(re.findall(r"\bk[A-Z]\w*", code))

problems = []
for doc in docs:
    text = doc.read_text(encoding="utf-8")
    rel = doc.relative_to(root)
    for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", text):
        if re.match(r"[a-z]+:", target):
            continue
        if not (doc.parent / target).exists():
            problems.append(f"{rel}: broken link {target}")
    for span in re.findall(r"`([^`\n]+)`", text):
        for name in re.findall(r"\bk[A-Z]\w*", span):
            if name not in known:
                problems.append(f"{rel}: `{name}` is not in src/ or tests/")

# Attribute names: from the table in code, each must be documented for users in the README.
names_h = "\n".join(l for l in (root / "src" / "sync" / "Box3DAttribNames.h").read_text(encoding="utf-8").splitlines()
                    if not l.lstrip().startswith("//"))     # the layout is described in comments
code_names = re.findall(r'\bX\(\s*\w+\s*,\s*"([^"]+)"', names_h) + re.findall(r'\bY\("([^"]+)"\)', names_h)
if not code_names:
    problems.append("src/sync/Box3DAttribNames.h: no attribute names found (did its X(...) layout change?)")
readme = (root / "README.md").read_text(encoding="utf-8")
readme_words = {w for span in re.findall(r"`([^`\n]+)`", readme) for w in re.findall(r"[A-Za-z_]\w*", span)}
for name in code_names:
    if name not in readme_words:
        problems.append(f"README.md: point attribute `{name}` (Box3DAttribNames.h) is not documented")

for p in sorted(set(problems)):
    print(p)
print(f"checked {len(docs)} files, {len(problems)} problem(s)")
sys.exit(1 if problems else 0)

#!/usr/bin/env python3
"""Check that the docs still point at things that exist.

For CLAUDE.md and docs/agents/*.md:
  * every relative markdown link resolves to a file;
  * every `kSomething` constant mentioned in backticks is still defined or used under src/ or tests/;
  * every point attribute named in src/sync/Box3DAttribNames.h is mentioned in backticks in the user docs
    (README.md and docs/*.md; the attribute table is prose, and this is what keeps it in step with the code).

Links are also checked in README.md and docs/*.md.

Docs name constants instead of copying their values, so a rename in code shows up here.
Usage: python tools/check_docs.py   (exit code 1 on any problem)
"""
import re
import sys
from pathlib import Path

root = Path(__file__).resolve().parent.parent
agent_docs = [root / "CLAUDE.md", *sorted((root / "docs" / "agents").glob("*.md"))]
user_docs = [root / "README.md", *sorted((root / "docs").glob("*.md"))]
docs = agent_docs + user_docs

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
        if re.match(r"[a-z]+:", target) or target == "../../releases":  # URLs, and GitHub's Releases page
            continue
        if not (doc.parent / target).exists():
            problems.append(f"{rel}: broken link {target}")
    for span in re.findall(r"`([^`\n]+)`", text) if doc in agent_docs else ():
        for name in re.findall(r"\bk[A-Z]\w*", span):
            if name not in known:
                problems.append(f"{rel}: `{name}` is not in src/ or tests/")

# Attribute names: from the table in code, each must be documented for users (README.md or docs/*.md).
names_h = "\n".join(line for line in (root / "src" / "sync" / "Box3DAttribNames.h").read_text(encoding="utf-8").splitlines()
                    if not line.lstrip().startswith("//"))     # the layout is described in comments
code_names = re.findall(r'\bX\(\s*\w+\s*,\s*"([^"]+)"', names_h) + re.findall(r'\bY\("([^"]+)"\)', names_h)
if not code_names:
    problems.append("src/sync/Box3DAttribNames.h: no attribute names found (did its X(...) layout change?)")
user_text = "\n".join(d.read_text(encoding="utf-8") for d in user_docs)
user_words = {w for span in re.findall(r"`([^`\n]+)`", user_text) for w in re.findall(r"[A-Za-z_]\w*", span)}
for name in code_names:
    if name not in user_words:
        problems.append(f"point attribute `{name}` (Box3DAttribNames.h) is not documented in README.md or docs/*.md")

for p in sorted(set(problems)):
    print(p)
print(f"checked {len(docs)} files, {len(problems)} problem(s)")
sys.exit(1 if problems else 0)

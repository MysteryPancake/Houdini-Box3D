# Documentation rules

Read this before editing `README.md`, `docs/*.md` or `docs/agents/*.md`. Index and code rules:
[CLAUDE.md](../../CLAUDE.md).

## Where things go

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and attributes exist, what they do,
and what behaves differently from Houdini's own solvers. Keep it short.
Leave out anything a user cannot act on or would not notice: how the solver does something, Box3D internals and terms
(density, restitution, move events, hull caches), performance mechanics. Those belong in `docs/agents/`.
When a change adds a feature, add one line or table row to the README if users need it, and put the explanation in the
matching `docs/agents/` file.

The README holds the install, build and quick start, and links to the user reference in `docs/` (`attributes.md`,
`shapes.md`, `static-objects.md`, `joints.md`, `nodes.md`, `notes.md`). The README rules below apply to all of them.

What is unverified, broken or still to do lives only in [status.md](status.md), [status-areas.md](status-areas.md) and
[todo.md](todo.md):
no TODO or "unverified" notes in code comments, the README or other docs.

## README rules (hard limits, not suggestions)

- The top part, from the title through the Building section, is the author's: never change it unless asked.
  Everything after it is user reference: short, organized, no history.
- Never add more than one or two sentences, or one table row, per change. Do not grow a section to explain a feature.
- No implementation details: no function, class or HDK names, locks, fingerprints, step ordering, clearing or caching
  behaviour. If a sentence says how the code works, it goes in `docs/agents/`.
- No semicolons: write separate sentences. No trailing asides with details a user does not need (which node sets a
  value, how it is stored, what else happens behind the scenes). Keep what they act on, cut the rest.
- Do not rewrite or expand existing README text while making an unrelated change. Trim, don't pad.
- Before finishing a task, re-read any README edit and cut it down.

## Rules for `docs/agents/` (and CLAUDE.md)

- Describe how the code works now, not how it got there: no history, no "was", "used to", "since fixed", "tried".
  A failed approach earns one clause only when it stops someone retrying it.
- Keep entries to a few lines; prefer a bullet to a paragraph. When something is settled, delete it from status.md or
  todo.md rather than marking it done.
- One fact in one place: link to the file that owns it instead of repeating it.
- Wrap lines at about 120 characters: one fact or sentence per line where it reads well. A bullet's continuation lines
  are indented two spaces. Never break a line inside a backtick span (`tools/dev/check_docs.py` reads spans line by line).
- Keep each file under about 12 KB. Split by topic when one grows past that, and update the index in CLAUDE.md.

## Tooltips

UI tooltips (the help text on parameters) follow the same rule: one or two short sentences, under about 150
characters. Say what the control does and, if needed, when to change it. No implementation detail, no lists of point
attributes, no restating how the sibling controls work.

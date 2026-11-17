# Documentation style

Rules for every README and every file in `docs/`. They keep the documentation uniform, short and reproducible.

## Contents

- [Basis](#basis)
- [Document structure](#document-structure)
- [Sentences](#sentences)
- [Commands, tables and lists](#commands-tables-and-lists)
- [Results](#results)
- [Formatting](#formatting)

## Basis

The rules follow conventions shared by the READMEs of widely used robotics repositories, read on 2026-11-17:
PX4-Autopilot, MuJoCo, Gazebo Sim, ns-3, Isaac Lab, Pinocchio, gym-pybullet-drones, ORB-SLAM3, VINS-Fusion, LIO-SAM,
AirSim, CARLA, Pegasus Simulator, RotorS, Nav2, MoveIt 2 and robosuite.

| Convention found in those repositories | Rule here |
|---|---|
| The first sentence defines the software: what it is and what it does | Rule 1 |
| A fixed order: overview, requirements, install, usage, documentation, citation, license | Rule 2 |
| Requirements name exact versions and the platform the software was tested on | Rule 3 |
| Long READMEs open with a table of contents | Rule 4 |
| Each command block follows a short sentence that says what it does | Rule 8 |
| Research code gives one runnable command per example | Rule 9 |
| The README stays short and links to detailed documents | Rule 5 |

Two conventions are not adopted. Badges are omitted because the project has no continuous integration. Promotional
wording ("state-of-the-art", "powerful") is omitted because every claim here must be measured.

## Document structure

1. Start with one `#` title, then one or two sentences that say what the document or directory is for.
2. Use this section order and omit sections that do not apply:
   Requirements, Build or Installation, Quick start or Usage, Reference, Results, Limitations, References.
3. State exact versions where behaviour depends on them, and the platform the item was tested on.
4. Add a `## Contents` list of section links to any document longer than about 100 lines.
5. Keep a README to an overview and link to `docs/` for detail. A directory README lists the directory's files in a
   table.
6. Put everything that is not implemented or not tested in a `## Limitations` section, in those words.

## Sentences

7. Write in the present tense and the active voice. Give instructions in the imperative ("Build the image").
   Keep one idea per sentence and prefer sentences under 25 words. Use the same term for the same thing throughout.

## Commands, tables and lists

8. Introduce every code block with a sentence that ends in a colon.
9. Put one command per line. Commands run from the repository root. Tag every block (`bash`, `yaml`, `json`, `text`).
10. Use a numbered list for steps, a table for anything enumerable (files, keys, arguments, criteria) and bullets for
    unordered facts. Keep a table cell to one sentence; move longer explanations below the table.

## Results

11. Report results only in `docs/EXPERIMENTS.md` or a scenario document, in a table with the date, configuration,
    command and outcome. Label a single run as a single run.
12. State pass criteria before results. Record every change to a criterion with its date and reason.
13. Keep development history and rationale out of `docs/`.

## Formatting

14. Wrap prose at 120 characters. Tables and code are exempt.
15. Use bold only for a term at the point it is defined. Use backticks for paths, commands, keys and values.
16. Use relative links between documents. Do not use emoji or badges.

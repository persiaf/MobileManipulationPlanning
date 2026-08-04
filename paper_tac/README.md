# TAC Manuscript Draft

This folder contains the IEEE TAC-oriented LaTeX draft for the RCH-MT-MMP project.

## Versions

- Archived baseline tag: `paper-tac-v0.1.0-baseline`
- Format-revision branch: `paper/tac-format-v0.2.0`
- Current project version: `0.2.0-format-revision`

## Main File

- `main.tex`: IEEEtran journal manuscript entry point.
- `sections/*.tex`: modular paper sections.
- `math_commands.tex`: shared notation.
- `references.bib`: cited bibliography entries only.
- `drafts/*.md`: Markdown-first section drafts for continued rewriting.
- `figures/`: local publication assets used by the revised manuscript.
- `docs/`: LaTeX structure, figure provenance, and revision records.
- `scripts/`: reproducible build and intermediate-file cleanup commands.

## Compile

From this directory, use a LaTeX environment with IEEEtran installed:

```bash
pdflatex main
bibtex main
pdflatex main
pdflatex main
```

or:

```bash
latexmk -pdf main.tex
```

The reproducible Windows command is:

```powershell
.\scripts\build_paper.ps1
```

The reproducible Linux/macOS command is:

```bash
./scripts/build_paper.sh
```

To remove LaTeX intermediate files while retaining the final PDF, sources, and
figures, run `.\scripts\clean_build.ps1` on Windows or
`./scripts/clean_build.sh` on Linux/macOS. The final PDF is written to
`build/main.pdf`.

## Figure check

Figure provenance and copy status are recorded in `docs/FIGURE_MANIFEST.md`.
Before release, check for missing assets with:

```bash
rg "MISSING" docs/FIGURE_MANIFEST.md
```

No output means the manifest has no asset marked `MISSING`.

## Evidence Sources

The manuscript cites figures from:

```text
results/figures/tac_submission/png/
```

The submission data report is:

```text
results/paper/TAC_DATA_SUPPORT_REPORT_SUBMISSION_20260629.md
```

The MATLAB entry point used to regenerate the submission-level data is:

```matlab
run_tac_data_support('submission')
```

The current draft intentionally excludes LNN as a main contribution and frames the paper around constrained hierarchical planning, reachability-aware docking, multi-docking coverage, and replaceable global routing.

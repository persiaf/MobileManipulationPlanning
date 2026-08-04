# TAC Manuscript Draft

This folder contains the IEEE TAC-oriented LaTeX draft for the RCH-MT-MMP project.

## Main File

- `main.tex`: IEEEtran journal manuscript entry point.
- `sections/*.tex`: modular paper sections.
- `math_commands.tex`: shared notation.
- `references.bib`: cited bibliography entries only.
- `drafts/*.md`: Markdown-first section drafts for continued rewriting.

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

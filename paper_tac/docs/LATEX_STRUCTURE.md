# LaTeX project structure

## Source layout

- `main.tex` is the IEEEtran journal entry point. It loads packages, shared
  notation, section files, and the bibliography.
- `sections/*.tex` contains one logical manuscript section per file. The order
  is controlled by the `\input{sections/...}` statements in `main.tex`.
- `math_commands.tex` defines shared mathematical notation and operators.
- `references.bib` contains the BibTeX records cited by the manuscript.
- `figures/` contains the publication assets referenced through the common
  `\graphicspath{{figures/}}` setting.

## Editing sections

To add a section, create a file under `sections/` and add a matching `\input`
at the intended location in `main.tex`. To remove a section from the compiled
paper, remove or comment its `\input`; retain the source file if it is still
needed for history. Reorder sections by moving only their `\input` statements.

## Adding figures

Place a single-column asset in `figures/` and use a standard `figure`:

```latex
\begin{figure}[!t]
    \centering
    \includegraphics[width=\columnwidth]{figure_name}
    \caption{Caption text.}
    \label{fig:figure_name}
\end{figure}
```

For a figure spanning both IEEE columns, use `figure*` and size panels relative
to `\textwidth`. Let LaTeX assign the figure number; do not call `\twocolumn` or
manually increment the figure counter.

## Building

Run from `paper_tac`:

```powershell
.\scripts\build_paper.ps1
```

or on Linux/macOS:

```bash
./scripts/build_paper.sh
```

Both scripts run `latexmk` with PDF output under `build/`. The expected final
file is `build/main.pdf`. The matching clean scripts remove intermediate files
while preserving the PDF and all source assets.

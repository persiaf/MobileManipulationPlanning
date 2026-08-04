#!/usr/bin/env sh
set -eu

paper_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$paper_root"
latexmk -c -outdir=build main.tex

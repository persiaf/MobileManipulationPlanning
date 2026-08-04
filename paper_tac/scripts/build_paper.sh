#!/usr/bin/env sh
set -eu

paper_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$paper_root"
mkdir -p build
latexmk -pdf -interaction=nonstopmode -synctex=1 -file-line-error -outdir=build main.tex

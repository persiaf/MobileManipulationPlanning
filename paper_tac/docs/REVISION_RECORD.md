# Revision record

## 0.2.0 format revision

- Limited the revision to project structure, IEEEtran layout, and figure
  management. Experimental values, table data, bibliography records, method
  conclusions, theory conclusions, and algorithm meaning were not rewritten.
- The archived `main.tex` contained a placeholder email and affiliation block,
  rather than the membership form anticipated by the task specification. The
  revised author block is conservatively reduced to `Peng Dai`.
- The archived four-panel `\twocolumn` implementation was already commented,
  while two active `figure*` blocks had mismatched panel captions and duplicate
  scientific captions. The revision removes the obsolete commented block and
  assigns each active pair its matching panel descriptions and labels.
- The archived reference-page break was commented rather than active. The
  obsolete commented command is removed so there is no forced reference break.
- After restoring IEEEtran float defaults, the two late double-column figures
  remain deferred to sparse float-only pages. Their definitions are kept next
  to the corresponding subsections, but no custom spacing, page-size change, or
  nonstandard figure counter is introduced to force placement in this pass.

No additional research-content issue was changed in this format-only pass.

# Maintenance Follow-up: 2026-08-31

## Scope

This follow-up keeps the MATLAB implementation and paper evidence intact. No complete MATLAB feature file was deleted, no GitHub push was performed, and no generated result file was removed.

## Test-scope correction

`tests/test_no_removed_learning_references.m` now scans maintained source roots, the root `README.md`, and `docs/**/*.md`. It intentionally excludes generated results, paper build directories, caches, archives, and historical artifacts. The check still fails when forbidden learning references are introduced into maintained source or documentation.

## Generated-artifact policy

The following paths are ignored because they are reproducible workspace products or historical learning artifacts:

- `paper_tac/paper_tac.zip`
- `paper_tac/results/cache/`
- `paper_tac/results/tmp/`
- `paper_tac/results/lnn/`

The rest of `paper_tac/results/` is not ignored wholesale. Paper-valid tables, raw data, figures, scripts, and manuscript notes remain reviewable and require explicit `KEEP` or `IGNORE` classification before release. No untracked result data was deleted in this round.

## Verification

- `run('tests/run_all_tests.m')`: PASS.
- `run('main_mobile_manipulation_planning.m')`: PASS.
- `run('experiments/exp_compare_hierarchical_strategies.m')`: PASS; MATLAB emitted PDF page-size warnings only.
- `visualize_single_algorithm` smoke check: PASS.
- Strategy regression: 36 tasks, 100% verified completion, 27 docks, task path 57.3436 m, return path 5.9512 m, total path 63.2948 m.
- `git diff --check`: PASS.

## Branch and GitHub status

- Current source branch: `paper/tac-format-v0.2.0`.
- Remote tracking branch: `origin/paper/tac-format-v0.2.0`.
- Local source branch is one commit ahead of its remote (`5bfd014`).
- Candidate planning branch for a future migration: `cleanup/matlab-maintenance-20260831`, based on `main`/`1a03b74`.
- Migration, conflict testing, authorization changes, and GitHub push were not performed in this round.

Before release, manually classify the remaining untracked `paper_tac/results/` contents, decide the target branch, review regenerated timing/figure changes, and then push only after that review.

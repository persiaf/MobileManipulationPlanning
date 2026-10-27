# S2-VIS audit

The inconsistent single-table figures came from `experiments/run_stage_progress_review_v1.m`, whose renderer reads `raw_data/stage_run.mat` and intentionally targets the C2.5.1 two-task fixture. They are not valid four-table results.

The corrected renderer reads the S2 `raw_data/rc2_plan.mat` file and iterates over every candidate-library region. The generated manifest reports 4 tables, 8 tasks, 1572 candidates, and 4 selected docks. The saved S2 path is empty because RC2 returned `BLOCKED`; the fourth figure states this explicitly.

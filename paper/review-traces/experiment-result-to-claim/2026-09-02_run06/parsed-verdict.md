# Parsed Verdict

- claim_supported: `yes`
- confidence: `high`
- status: provisional - no paper claim audit available
- paper_claim_audit: `unavailable`
- supported claim: In the tested S4 seeds `[1, 3, 5]`, current Bidirectional/`convex_astar` configuration, the run-local cache preserves all checked outputs, removes about 75% of expensive calls, and yields about 3.7x mean paired speedup. The same-batch Position-only-plus-cache configuration achieves 100% coverage and lower wall-clock than uncached Baseline in all six pairs, with about 2.0x mean speedup.
- unsupported claim: General speedup, path-quality superiority, augmentation-alone runtime reduction, or transfer to other planners, hardware, layouts, and scales.
- evidence limitation: two repeats, three seeds, candidate generation measured once per seed, and aggregate rather than query-level call auditing.

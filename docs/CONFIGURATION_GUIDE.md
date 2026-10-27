# Configuration Guide

Effective order: BASE CONFIG -> PROFILE -> EXPERIMENT OVERRIDE -> USER/GUI OVERRIDE -> EFFECTIVE CONFIG.

Base files: config/scene_config.m, config/robot_config.m, config/planner_config.m, and visualization configuration. Named profiles are resolved by experiment_config_builder or config/resolve_effective_config.m. Use overrides for routing mode, sparse k, inflation, candidate augmentation, reachability, footprint, return-to-start, and visualization. A default is not the same as a validated experiment override. Preserve geometry, reachability limits, candidate generation, and frozen profiles when reproducing results.

This directory contains a few scripts for making convergence plots.

## Plot detailed convergence data

`plot_convergence_data.py` makes a `matplotlib` plot of open-loop convergence
data for a given example.

Usage:
- Run an example, e.g., `bazel run //examples/kuka:kuka`
    - Make sure the example YAML is configured for open-loop optimization
      (`run_mpc : false`) and `save_solver_stats_csv : true`.
- Run the plotting script, specifying the example as a command line argument,
  e.g.,
```
$ ./scripts/plot_convergence_data.py kuka
```


## Compare convergence with different settings

`compare_convergence.py` makes a less detailed plot comparing two or more
open-loop optimizations with the same example but different settings.

Usage:
- Run an example, e.g., `bazel run //examples/acrobot:acrobot`
    - Make sure the example YAML is configured for open-loop optimization
      (`run_mpc : false`) and `save_solver_stats_csv : true`.
    - The resulting CSV file is stored in
      `bazel-out/k8-opt/bin/examples/acrobot/acrobot.runfiles/idt/solver_stats.csv`.
      Make a copy of this file with a new name, e.g,
      `solver_stats_trust_region.csv`, also in `bazel-out/k8-opt/...`
- Run the example again with different settings
    - For example, set `method : linesearch`
    - Make a copy of `solver_stats.csv`, e.g. `solver_stats_linesearch.csv`. 
- Repeat as many times as you like with various settings, each time making a new
  copy of `solver_stats.csv`. 
- Edit `compare_convergence.py` to set `example_name`, `csv_names`, and `labels`
  to match the example you used, the new CSV names you created, and the labels
  you want to show up on the plot
- Run the plotting script:
```
$ ./scripts/compare_convergence.py
```

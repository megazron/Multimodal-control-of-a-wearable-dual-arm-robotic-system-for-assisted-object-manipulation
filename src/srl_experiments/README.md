# srl_experiments — the studies

Runner, conditions, logging and analysis for E1–E5. Depends on everything;
nothing depends on it.

    experiments/
      e1_fitts_characterisation/   protocol.md config.yaml run_fitts.py            analyse_fitts.py            scenarios/ results/
      e2_autonomy_level/           protocol.md config.yaml run_autonomy_level.py   analyse_autonomy_level.py   scenarios/ results/
      e3_divided_attention/        protocol.md config.yaml run_divided_attention.py analyse_divided_attention.py scenarios/ results/
      e4_dof_recovery/             protocol.md config.yaml run_dof_recovery.py     analyse_dof_recovery.py     scenarios/ results/
      e5_intent_inference/         protocol.md config.yaml run_intent_inference.py analyse_intent_inference.py scenarios/ results/

## Run

    bash scripts/run_experiment.sh e1 --participant P01
    bash scripts/run_experiment.sh e1 --participant PILOT --scripted   # no human

`--scripted` drives the master from `scripted_operator`, so every experiment
runs end to end with no hardware and no participant. That is how the pipeline
is regression-tested.

## Which need people

| | needs participants |
| --- | --- |
| E1 Fitts characterisation | no — system evaluation |
| E5 Intent inference characterisation | no — system evaluation |
| E2 autonomy level | **yes** |
| E3 divided attention | **yes** |
| E4 DOF recovery | **yes** |

## Data

One CSV per trial plus a session manifest, under
`experiments/<exp>/results/<participant>/`. Participant IDs are anonymised
codes; **no name, email or date of birth appears in any file**. See
`docs/research/03_ethics_and_safety.md`.

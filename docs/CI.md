# Module continuous integration

The workflows in [`.github/workflows/`](../.github/workflows/) test the existing
synthetic Python modules. Each workflow runs on Ubuntu 24.04 with Python 3.12
and 3.14. Relevant pull requests and pushes to `main` trigger the checks;
`workflow_dispatch` also permits manual runs once the workflow is on the default
branch. A feature-branch push alone does not trigger a run.

| Workflow | Existing coverage |
| --- | --- |
| CV CI | CV observation interface, perception, and benchmark tests |
| Motion CI | All tests under `motion_engine/tests`, including failure injection and trace checks |
| State Machine CI | Mission transitions, demos, and Motion/state-machine alignment |
| Safety CI | Policy, legacy decisions, concrete command checks, and the Motion adapter |
| Universal Log CI | Shared logger output, serialization, and Motion/CV logging interoperability |
| Docking CI | Alignment, phases, replay, hazard cancellation, and Safety policy/command checks |

Navigation and Simulation currently contain documentation and placeholders, with
no runnable module tests. Hardware and Electronics also have no Python test
suites. Add CI jobs for those modules when they have a meaningful runnable check.
These workflows do not install PX4, ROS, Gazebo, or external submodules, and do
not change mission behavior or deploy anything.

## Dependencies and local reproduction

Docking's unittest suites need only the standard library. The pytest-based jobs
install [requirements-ci.txt](../requirements-ci.txt); CV additionally installs
its existing [requirements](../cv/requirements.txt). Separate jobs prevent CV
dependencies from being required for other modules. Run commands from the
repository root in an isolated environment:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-ci.txt -r cv/requirements.txt

# CV
python -B -m unittest discover -s tests -p 'test_computer_vision.py' -v
python -B -m unittest discover -s tests -p 'test_cv*.py' -v

# Motion
python -B -m pytest motion_engine/tests -v

# State Machine
python -B -m pytest state_machine/tests motion_engine/tests/test_state_machine_alignment.py -v

# Safety
python -B -m unittest discover -s tests -p 'test_safety*.py' -v
python -B -m unittest discover -s tests -p 'test_command_check.py' -v
python -B -m pytest motion_engine/tests/test_safety_adapter.py -v

# Universal Log
python -B -m pytest motion_engine/tests/test_universal_logging.py -v

# Docking (includes its Safety dependencies)
python -B -m unittest discover -s tests -p 'test_docking*.py' -v
python -B -m unittest discover -s tests -p 'test_command_check.py' -v
python -B -m unittest discover -s tests -p 'test_safety_policy.py' -v
```

## Viewing results

Open the repository's **Actions** tab and choose a module workflow. Each run
contains separate Python-version jobs and named test steps. Pull requests also
show the runs under **Checks**. A failure reports the test and traceback;
subsequent test steps still run unless the workflow was cancelled.

Path filters include each module's imported local dependencies, its test files,
and workflow/configuration changes. If an import dependency moves, update the
filters too. These are scoped checks: unrelated changes can skip a workflow.
Do not mark a path-filtered workflow as a universally required branch-protection
check without accounting for skipped runs. No branch protection is configured
by this change.

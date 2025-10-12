# Contributing

Thank you for contributing to PyPnC. A couple of conventions help keep the codebase clean.

## Install development tools

We use `pre-commit` to run formatters and linters before commits. Install them locally once:

```bash
python3 -m pip install --user pre-commit black isort flake8 mypy
# or using your virtualenv: pip install pre-commit black isort flake8 mypy
```

## Enable git hooks

Install the git hooks (run once per clone):

```bash
pre-commit install
```

This will automatically run configured hooks on `git commit` (the repository config limits checks to `pypnc/`).

## Run checks manually

- Run only changed files (default behavior on commit):

```bash
pre-commit run
```

- Run all checks across the repository (useful before opening a PR):

```bash
pre-commit run --all-files
```

- Equivalent Makefile targets:

```bash
make precommit-install  # installs hooks
make precommit-run      # runs pre-commit --all-files
```

If you want CI parity, run the same tools locally as used in the workflow:

```bash
make lint-quick
make typecheck-quick
pytest -q
```

Thanks — small steps keep the code healthy and reviewable.

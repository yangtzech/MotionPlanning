## Makefile for $(QuickCheckFolder) developer helper tasks

PY=python3
QuickCheckFolder=Control

.PHONY: help quick full test lint typecheck

help:
	@echo "Makefile targets:"
	@echo "  make quick    - quick checks ($(QuickCheckFolder) only): flake8, mypy, pytest"
	@echo "  make full     - full checks (whole repo): flake8, mypy, pytest"
	@echo "  make test     - run pytest"
	@echo "  make lint     - run flake8"
	@echo "  make typecheck - run mypy"

# Quick mode: only check package under development
quick: lint-quick typecheck-quick test-quick

lint-quick:
	$(PY) -m flake8 $(QuickCheckFolder)

typecheck-quick:
	@echo "Running mypy (quick) against $(QuickCheckFolder)..."
	$(PY) -m mypy $(QuickCheckFolder) 2>&1 | tee mypy-output.txt

test-quick:
	$(PY) -m pytest -q

# Full mode: run checks across the entire repository
full: lint-full typecheck-full test-full

lint-full:
	$(PY) -m flake8

typecheck-full:
	$(PY) -m mypy 2>&1 | tee mypy-output.txt

test-full:
	$(PY) -m pytest -q

lint: lint-quick
typecheck: typecheck-quick
test: test-quick

# Combined checks target: lint + typecheck + tests (used in CI or locally)
checks: lint typecheck test
## Makefile for PyPnC developer helper tasks

PY=python3

.PHONY: help quick full test lint typecheck

help:
	@echo "Makefile targets:"
	@echo "  make quick    - quick checks (pypnc only): flake8, mypy, pytest"
	@echo "  make full     - full checks (whole repo): flake8, mypy, pytest"
	@echo "  make test     - run pytest"
	@echo "  make lint     - run flake8"
	@echo "  make typecheck - run mypy"

# Quick mode: only check package under development
quick: lint-quick typecheck-quick test-quick

lint-quick:
	$(PY) -m flake8 pypnc

typecheck-quick:
	$(PY) -m mypy pypnc

test-quick:
	$(PY) -m pytest -q

# Full mode: run checks across the entire repository
full: lint-full typecheck-full test-full

lint-full:
	$(PY) -m flake8 pypnc

typecheck-full:
	$(PY) -m mypy pypnc

test-full:
	$(PY) -m pytest -q

lint: lint-full
typecheck: typecheck-full
test: test-full

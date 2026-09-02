PYTHON = python3
MAP = maps/easy_linear.txt

.PHONY: install venv run debug clean lint lint-strict

install:
	$(PYTHON) -m pip install -r requirements.txt

venv:
	$(PYTHON) -m venv .venv

run:
	$(PYTHON) main.py $(MAP)

debug:
	$(PYTHON) -m pdb main.py $(MAP)

clean:
	rm -rf __pycache__ .mypy_cache

lint:
	$(PYTHON) -m flake8 .
	$(PYTHON) -m mypy . \
		--warn-return-any \
		--warn-unused-ignores \
		--ignore-missing-imports \
		--disallow-untyped-defs \
		--check-untyped-defs

lint-strict:
	$(PYTHON) -m flake8 .
	$(PYTHON) -m mypy . --strict

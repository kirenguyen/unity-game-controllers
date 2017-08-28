init:
	pip install -r requirements.txt
test:
	nosetests -v --nocapture tests

lint:
	pylint TapGameController/*.py TapGameController/**/*.py

lev_matrix:
	python -m scripts.generate_levenshtein_weight_matrix
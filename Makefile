init:
	pip install -r requirements.txt
	python -m spacy download en

test:
	nosetests -v --nocapture tests

lint:
	pylint TapGameController/*.py TapGameController/**/*.py

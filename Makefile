init:
	pip install -r requirements.txt

test:
	nosetests -v --nocapture tests

lint:
	pylint TapGameController/*.py TapGameController/**/*.py

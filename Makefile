init:
	pip install -r requirements.txt

test:
	nosetests tests

lint:
	pylint TapGameController/*.py TapGameController/**/*.py

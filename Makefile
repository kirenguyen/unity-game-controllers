init:
    @echo "Installing dependencies from requirements.txt"
	pip install -r requirements.txt

test:
	nosetests -v --nocapture tests

lint:
    @echo "Running PyLint on all TapGameController Files"
	pylint TapGameController/*.py TapGameController/**/*.py

curriculum:
    @echo "Generating curriculum from GameUtils/Curriculum.py"
    python -m scripts.generate_levenshtein_weight_matrix

lev_matrix: curriculum
    @echo "Generating lev_matrix from curriculum"
	python -m scripts.generate_levenshtein_weight_matrix

glove_matrix: curriculum
    @echo "Generating glove_matrix from curriculum"
	python -m scripts.generate_glove_matrix

covariance_matrix: lev_matrix glove_matrix
    @echo "Generating covariance matrix from glove_matrix and lev_matrix"
	python -m scripts.generate_covariance_matrix
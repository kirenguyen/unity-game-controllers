"""
This Module handles all aspects of student modeling and assessment
"""
from random import randint
import numpy as np
from GameUtils.Curriculum import Curriculum

class StudentModel():
    """
    This class implements a Gaussian Process, intended to model student vocabulary knowledge
    It uses a kernel based on the distance between two words in ConceptNet as well as a
    phonetic distance heuristic
    """

    def __init__(self):
        #fancy python one-liner to read all string attributes off of a class
        self.curriculum = [p for p in dir(Curriculum)
                           if isinstance(getattr(Curriculum, p), str)
                           and not p.startswith('__')]

    def init_model(self):
        """
        sets up the GP and kernel
        """
        # Test data
        # we only care about evaluating the GP at this finite collection of points
        x_test = self.curriculum

        cov_ss = concept_net_kernel(x_test, x_test) #the Kernel Matrix for the words
        print(cov_ss)

        # Get cholesky decomposition (square root) of the
        # covariance matrix
        chol_matrix = np.linalg.cholesky(cov_ss + 1e-15 * np.eye(len(x_test)))

        # Sample 3 sets of standard normals for our test points,
        # multiply them by the square root of the covariance matrix
        f_prior = np.dot(chol_matrix, np.random.normal(size=(len(x_test), 3)))
        print(f_prior)


    def get_next_best_word(self):
        """
        gives an external caller the next best word to achieve some objective
        Active Learning paradigm should be implemented here!
        """    
        return self.curriculum[randint(0, len(self.curriculum) - 1)] #randint is inclusive

    def add_data_point(self, word_asked, answered_correct):
        """
        allows external controller to update the GP with new data
        """
        print(self)
        print('answered ' + word_asked+ ('in' if not answered_correct else '') + 'correctly')

        # # Noiseless training data
        # Xtrain = np.array([-4, -3, -2, -1, 1]).reshape(5, 1)
        # ytrain = np.sin(Xtrain)
        #
        # # Apply the kernel function to our training points
        # K = kernel(Xtrain, Xtrain, param)
        # L = np.linalg.cholesky(K + 0.00005 * np.eye(len(Xtrain)))
        #
        # # Compute the mean at our test points.
        # K_s = kernel(Xtrain, Xtest, param)
        # Lk = np.linalg.solve(L, K_s)
        # mu = np.dot(Lk.T, np.linalg.solve(L, ytrain)).reshape((n,))
        #
        # # Compute the standard deviation so we can plot it
        # s2 = np.diag(K_ss) - np.sum(Lk ** 2, axis=0)
        # stdv = np.sqrt(s2)
        # # Draw samples from the posterior at our test points.
        # L = np.linalg.cholesky(K_ss + 1e-6 * np.eye(n) - np.dot(Lk.T, Lk))
        # f_post = mu.reshape(-1, 1) + np.dot(L, np.random.normal(size=(n, 3)))
        # print(f_post.shape)

        # pl.plot(Xtrain, ytrain, 'bs', ms=8)
        # pl.plot(Xtest, f_post)
        # pl.gca().fill_between(Xtest.flat, mu - 2 * stdv, mu + 2 * stdv, color="#dddddd")
        # pl.plot(Xtest, mu, 'r--', lw=2)
        # pl.axis([-5, 5, -3, 3])
        # pl.title('Three samples from the GP posterior')
        # pl.show()

def rbf_kernel(input_a, input_b, length_scale):
    """
    Implements to Radial Basis Function kernel
    """

    sqdist = np.sum(input_a ** 2, 1).reshape(-1, 1) + \
             np.sum(input_b ** 2, 1) - 2 * np.dot(input_a, input_b.T)
    return np.exp(-.5 * (1 / length_scale) * sqdist)

def concept_net_kernel(word_set_a, word_set_b):
    """
    Implements a conceptnet based covariance matrix
    """
    k = np.empty((len(word_set_a), len(word_set_b),))
    k[:] = .7 #for now, every element is .7
    return k

"""
This Module handles all aspects of student modeling and assessment
Some code remixed from
http://katbailey.github.io/post/gaussian-processes-for-dummies/
^ Great intro article for rolling your own GP
"""
from random import randint
import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
from .TapGameUtils.Curriculum import Curriculum

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


    def get_prior(self, n_samples):
        """
        Samples form a prior distribution over the words
        """

        # Test data
        # we only care about evaluating the GP at this finite collection of points
        x_test = self.curriculum

        cov_ss = self.concept_net_kernel(x_test, x_test) #the Kernel Matrix for the words
        print(cov_ss)

        # Get cholesky decomposition (square root) of the
        # covariance matrix
        chol_matrix = np.linalg.cholesky(cov_ss + 1e-15 * np.eye(len(x_test)))

        # Sample 3 sets of standard normals for our test points,
        # multiply them by the square root of the covariance matrix
        f_prior = np.dot(chol_matrix, np.random.normal(size=(len(x_test), n_samples)))
        return f_prior



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


    def concept_net_kernel(self, word_set_a, word_set_b):
        """
        Implements a conceptnet based covariance matrix
        """
        #k = np.empty((len(word_set_a), len(word_set_b),))

        #first, validate that these are all words in our curriculum
        for elem in set().union(word_set_a, word_set_b):
            if not elem in self.curriculum:
                raise Exception(elem + ' is not a word in our curriculum')

        k = np.ones((len(word_set_a), len(word_set_b)))
        for i in range(len(word_set_a)):
            for j in range(len(word_set_b)):
                k[i][j] = self.get_word_cov(word_set_a[i], word_set_b[j])

        return k

    def get_word_cov(self, word_a, word_b):
        """
        Currently, just implements a letter counting distance metric.
        Will eventually incorporate conceptnet and other phonetic metrics
        """

        if (word_a == word_b):
            return 1

        score = 0
        for letter_a in word_a:
            if letter_a in word_b:
                score += 1

        for letter_b in word_b:
            if letter_b in word_a:
                score += 1

        ratio = (score / (len(word_a) + len(word_b)))
        return round(ratio, 2)

    def plot_curricular_distro(self, means, vars):

        n_rows = 2 # needs to be > 1

        # f, plts = plt.subplots(n_rows, int(test_space_size / n_rows), sharex='col', sharey='row', figsize=(15,10))
        f, plts = plt.subplots(n_rows, int(len(self.curriculum) / n_rows), figsize=(15, 10))
        # print(plts)


        left = 0.125  # the left side of the subplots of the figure
        right = 0.9  # the right side of the subplots of the figure
        bottom = 0.4  # the bottom of the subplots of the figure
        top = 0.9  # the top of the subplots of the figure
        wspace = 1.0  # the amount of width reserved for blank space between subplots
        hspace = 0.2  # the amount of height reserved for white space between subplots

        for i in range(len(self.curriculum)):
            row_index = int(i / (len(self.curriculum) * .5))
            col_index = int(i % (len(self.curriculum) / n_rows))
            plts[row_index][col_index].set_xlim([-3, 3])
            plts[row_index][col_index].set_ylim([-1.5, 1.5])

            #data = f_post[:][i]
            print(self.curriculum[i])
            #print(np.mean(data))
            #print(np.var(data))
            print(means[i])
            print(vars[i])
            # plts[row_index][col_index].scatter(data, np.zeros(n_samples))

            x = np.linspace(-3, 3, 50)
            plts[row_index][col_index].plot(x, scipy.stats.norm.pdf(x, means[i], vars[i]))
            plts[row_index][col_index].set_title(self.curriculum[i])

        plt.subplots_adjust(left, bottom, right, top, wspace, hspace)



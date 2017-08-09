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
from .TapGameUtils.PronunciationUtils import PronunciationHandler

class StudentModel(): # pylint: disable=invalid-name,consider-using-enumerate

    """
    This class implements a Gaussian Process, intended to model student vocabulary knowledge
    It uses a kernel based on the distance between two words in ConceptNet as well as a
    phonetic distance heuristic.
    pylint's 'invalid-name' checker has been disabled to conform with the notation
    in Rasmussen and Williams
    """

    def __init__(self):
        #fancy python one-liner to read all string attributes off of a class
        self.curriculum = [p for p in dir(Curriculum)
                           if isinstance(getattr(Curriculum, p), str)
                           and not p.startswith('__')]

        # these parameters govern the assumed Gaussian noise added to the child's recorded
        # pronunciation assessment
        self.noise_mu = 0
        self.noise_sigma = .3

        self.X_train = [] # These are persistent lists of training data!
        self.Y_train = []

        self.means = [.5] * len(self.curriculum)  # These are the most recent posteriors
        self.variances = [1] * len(self.curriculum) # Together they form the Student Model!
        self.fig = None # Figure for drawing
        self.plts = None #Plots


    def init_model(self):
        """
        sets up the GP and kernel
        """


    def get_prior(self, n_samples):
        """
        Samples form a prior distribution over the words
        See Algorithm 2.1 in Rasmussen and Williams to follow along with implementation + notation
        """

        # Test data
        # we only care about evaluating the GP at this finite collection of points
        x_test = self.curriculum

        K_ss = self.concept_net_kernel(x_test, x_test) #the Kernel Matrix for the words

        # Get cholesky decomposition (square root) of the
        # covariance matrix
        L = np.linalg.cholesky(K_ss + 1e-15 * np.eye(len(x_test)))

        # Sample 3 sets of standard normals for our test points,
        # multiply them by the square root of the covariance matrix
        f_prior = np.dot(L, np.random.normal(size=(len(x_test), n_samples)))
        return f_prior

    def train_and_compute_posterior(self, new_X_train, new_Y_train):
        """
        Takes in a list of labeled X and labeled Y data points and computes a new posterior
        E.g.
        new_X_train = ['BEE', 'SNAKE', 'TIGER']  # these numbers are the word labels
        new_Y_train = [1, 1, 1] # these numbers correspond to full correct pronunciations

        See Algorithm 2.1 in Rasmussen and Williams to follow along with implementation + notation
        """

        Xtest = self.curriculum  # these numbers are just labels

        for i in range(len(new_X_train)):
            self.X_train.append(new_X_train[i])
            self.Y_train.append(new_Y_train[i])

        # compute cov of train set wrt itself + cov of all words wrt all words
        K = self.concept_net_kernel(self.X_train, self.X_train)
        K_ss = self.concept_net_kernel(Xtest, Xtest)

        L = np.linalg.cholesky(K + 0.00005 * np.eye(len(self.X_train)) +
                               ((self.noise_sigma ** 2) * np.eye(len(self.X_train))))

        L_y = np.linalg.solve(L, self.Y_train)
        a = np.linalg.solve(L.T, L_y)

        # Compute the mean at our test points.
        K_s = self.concept_net_kernel(self.X_train, Xtest)
        mu = np.dot(K_s.T, a).reshape(len(self.curriculum), )
        v = np.linalg.solve(L, K_s)

        # we only want the diagonal bc we want to know
        # variance of each variable independent of any others
        variance = np.diag(K_ss - np.dot(v.T, v))
        #stdv = np.sqrt(variance)

        self.means = mu
        self.variances = variance
        return mu, variance


    def get_next_best_word(self):
        """
        gives an external caller the next best word to achieve some objective
        Active Learning paradigm should be implemented here!
        """
        return self.curriculum[randint(0, len(self.curriculum) - 1)] #randint is inclusive

    def rbf_kernel(self, input_a, input_b, length_scale):
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

        # if word_a == word_b:
        #     return 1

        # score = 0
        # for letter_a in word_a:
        #     if letter_a in word_b:
        #         score += 1

        # for letter_b in word_b:
        #     if letter_b in word_a:
        #         score += 1

        # ratio = (score / (len(word_a) + len(word_b)))
        # return round(ratio, 2)
        self.pronunciationHandler = PronunciationResultsHandler()
        return self.pronunciationHandler.measure_weighted_levenshtein_distance(word_a,word_b)

       


    def plot_curricular_distro(self):
        """
        Plots the most recent distribution
        """

        n_rows = 2 # needs to be > 1

        # f, plts = plt.subplots(n_rows, int(test_space_size / n_rows),
        #                        sharex='col', sharey='row', figsize=(15,10))
        fig, plts = plt.subplots(n_rows, int(len(self.curriculum) / n_rows), figsize=(15, 10))
        self.fig = fig
        self.plts = plts
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
            self.plts[row_index][col_index].set_xlim([-3, 3])
            self.plts[row_index][col_index].set_ylim([-1.5, 1.5])

            #data = f_post[:][i]
            #print(self.curriculum[i])
            #print(np.mean(data))
            #print(np.var(data))
            #print(self.means[i])
            #print(self.variances[i])
            # plts[row_index][col_index].scatter(data, np.zeros(n_samples))

            x = np.linspace(-3, 3, 50)
            self.plts[row_index][col_index].plot(x, scipy.stats.norm.pdf(x, self.means[i],
                                                                         self.variances[i]))
            self.plts[row_index][col_index].set_title(
                self.curriculum[i] + ": u= " + str(round(self.means[i], 2)) + ", var= " + str(
                    round(self.variances[i], 2)))

        plt.subplots_adjust(left, bottom, right, top, wspace, hspace)
        plt.show()
        plt.draw()

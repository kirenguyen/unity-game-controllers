import pronouncing
import csv
import os

class PhonemeCounter():
	def __init__(self):
		self.load_nettalk_dataset()
		self.phonemes2graphemes("apple")

	def load_nettalk_dataset(self):
		"""
		nettalk contains phonetic transcription of 20,008 english words
		dataset can be accessed at:
	https://archive.ics.uci.edu/ml/machine-learning-databases/undocumented/connectionist-bench/nettalk/
		"""

		dir_path = os.path.dirname(os.path.realpath(__file__))

		self.nettalk_dataset = dict()
		with open(dir_path + '/data/nettalk.data', 'r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter='\t')
			for row in spamreader:
				# row[0]: word. row[1]: phonetic transcription
				self.nettalk_dataset.update({row[0]: row[1]})

	def phonemes2graphemes(self, word):
		"""
		align a given word's graphemes with its phonemes using Nettalk
		"""

		print(word)
		print(pronouncing.phones_for_word(word.lower()))

		# need to use lowercase form of word for pronouncing and nettalk dict lookup
		phonemes_raw = pronouncing.phones_for_word(word.lower())[0].split(' ')
		phonemes = [''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]

		# find the phoneme-grapheme alignment in Nettalk database
		# get exact alignment
		phos = self.nettalk_dataset[word.lower()]

		print(phos)

		phos = list(phos)
		graphemes = list()
		grapheme = ''
		for i in range(0, len(phos), 1):
			if phos[i] != '-':
				# one phoneme is matched to one letter
				if grapheme != '':
					graphemes.append(grapheme)
				grapheme = word[i]
			else:
				# one phoneme is matched to an additional letter
				grapheme += word[i]
		graphemes.append(grapheme)

		# TODO: This is hack-ey and should not be done. Let this comment stand as a reminder
		# TODO: that sometimes phonemes and graphemes dont line up right...

		# For some reason camera doesn't line up phonemes and graphemes correctly
		# if word == "camera":
		#	graphemes = ['c', 'a', 'm', 'er', 'a']

		print([graphemes, phonemes])
		return [graphemes, phonemes]

	# def count_phonemes(self)

if __name__ == '__main__':
	counter = PhonemeCounter()














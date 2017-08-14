import pronouncing
import csv
import os
import pandas as pd 
import numpy as np 

SCORE_THRESHOLD = 65

class PronunciationResultsHandler:
	'''
	This class contains functions that handles pronunciation results obtained from SpeechACE
	'''

	def __init__(self):
		# loads arpabet mapping, an IPA-like representation of phonemes for computer processing
        self.load_arpabet_mapping()

        # load nettalk.data dictionary, which is used to alignment word graphemes and phonemes
        self.load_nettalk_dataset()

        self.current_word = None
        self.pho_results = None
        self.syl_results = None
	

	def process_Speechace_results(self,results):
		'''
		takes in results from speechace and analyze the user's pronunciation accuracy
		'''
		if word_results is None:
            return [[''], ['0']]
        else:
			self.current_word = results["word"]
			self.pho_results = results["phone_score_list"]
			self.syl_results = results["syllable_score_list"]

		phoneme_acc = self.phoneme_based_accuracy(self.pho_results)
		# syllable_acc = self.syllable_based_accuracy(self.syl_results)


		return self.get_letters_and_pass(phoneme_acc)


	def phoneme_based_accuracy(self,phoneme_results):
		''''
		checks which graphemes pass the accuracy threshold
		return: a tuple that contains three lists (graphemes,phonemes and bool values indicating whether the graphemes pass threshold)
		'''
		
		graphemes,phonemes = self.phonemes2graphemes(self.current_word)
		pass_list = []

		for i in range(len(phonemes)):
			score = self.pho_results[i]["quality_score"]
			if score >= SCORE_THRESHOLD:
				pass_list.append('1')
			else:
				pass_list.append('0')

		return (graphemes,phonemes,pass_list)

	def syllable_based_accuracy(self,syllable_results):
		'''
		check which syllables pass the accuracy threshold
		return: a tuple that contains two lists (syllables and bool values)
		'''
		syllable_list = []
		pass_list = []

		for i in range(len(self.syl_results)):
			syllable_list.append(self.syl_results[i]["letters"])
			score = self.syl_results[i]["quality_score"]
			if score >= SCORE_THRESHOLD:
				pass_list.append('1')
			else:
				pass_list.append('0')

		return (syllable_list, pass_list)

	def get_letters_and_pass(self, phoneme_acc):
		'''
		should be called by iSpy Game Controller to get the final pronunciation message for ROS
		return: a dictionary of letters with bool values indicating whether letters should be red or greeen
		'''
		
		# Uncomment for phoneme accuracy
		graphemes, phonemes, scores = phoneme_acc
		letters = []
		passed = []

		for i in range(len(graphemes)):
			if len(graphemes[i]) > 1:
				for j in graphemes[i]:
					letters.append(j)
					passed.append(scores[i])
			else:
				letters.append(graphemes[i])
				passed.append(scores[i])

		return letters, passed
		
	def load_arpabet_mapping(self):

		dir_path = os.path.dirname(os.path.realpath(__file__))

		# dictionary to store arpabet mapping 
		self.arpabet_map = dict()
		with open(dir_path + '/arpabet_mapping.csv','r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				#row[1] is CMU arpabet, row[0] is a modified version of arpabet for NETTalk database
 				self.arpabet_map.update({row[1]:row[0]})


	def load_nettalk_dataset(self):
		''' 
		nettalk contains phonetic transcription of 20,008 enlish words
 		dataset can be accessed: https://archive.ics.uci.edu/ml/machine-learning-databases/undocumented/connectionist-bench/nettalk/
		'''

		dir_path = os.path.dirname(os.path.realpath(__file__))

		self.nettalk_dataset=dict()
		with open(dir_path + '/nettalk.data','r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter='\t')
			for row in spamreader:
				# row[0]: word. row[1]: phonetic transcription
				self.nettalk_dataset.update({row[0]:row[1]})


	def phonemes2graphemes(self,word):
		'''
		align a given word's graphemes with its phonemes
		'''
		phonemes_raw=pronouncing.phones_for_word(iword)[0].split(' ')
		phonemes=[''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
		
		#find the phoneme-grapheme alignment in Nettalk database
		#get exact alignment
		phos=self.nettalk_dataset[word]
		
		# print(phos)
	
		phos=list(phos)
		word_list = list(word)
		graphemes = list()
		grapheme=''
		for i in range(0,len(phos),1):
			if phos[i] != '-':
				# one phoneme is matched to one letter
				if grapheme!='':
					graphemes.append(grapheme)
				grapheme = word[i]
			else:
				# one phoneme is matched to an additional letter
				grapheme+=word[i]
		graphemes.append(grapheme)

		# For some reason camera doesn't line up phonemes and graphemes correctly
		if word == "camera":
			graphemes = ['c', 'a', 'm', 'er', 'a']

		# print([graphemes,phonemes])
		return [graphemes,phonemes]

	#def convert_arpabet_to_unique_ids(self):
	# 	'''
	# 	maps phonemes from CMU arpabet to unique index numbers 
	# 	'''
	# 	self.arpabet_ids = dict()
		
	def conversion_for_phonetic_similarity(self,iword):
		'''
		convert a given word into a unique phonetic transcription, 
		which allows for measuring phonetic siimlarity with other words
		output: a phonetic string for the input word. each letter in the string corresponds uniquely to a phone
		'''
		phonemes_raw=pronouncing.phones_for_word(iword)[0].split(' ')
		phonemes=[''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
		print(phonemes_raw)
		output = ''
		for phoneme in phonemes:
			if phoneme in self.arpabet_map:
				output+=self.arpabet_map[phoneme]
			else:
				print("phone ( "+phoneme+" ) does not exist in arpabet map")
				break
		# print(output)
		return output

	def measure_weighted_levenshtein_distance(self,word1,word2):
		# import weighted levenshtein library. if clev is missing, change the __init__.py in the weighted_levenshtein lib to add clev.so path to sys.
		from weighted_levenshtein.clev import levenshtein as lev

		substitute_costs = np.ones((128, 128), dtype=np.float64)  # make a 2D array of 1's. ASCII table
		
		# read weighted phonemic similarity matrix, 
		# downloaded from https://github.com/benhixon/benhixon.github.com/blob/master/wpsm.txt
		wpsm_filepath = 'wpsm.csv'
		# load the matrix csv file into a dataframe
		df = pd.read_csv(wpsm_filepath,sep=',', header =0,index_col=0)
		
		arpabet_phonemes=df.keys()

		#check whether arpabet map is empty. if it is, then load the map
		if not self.arpabet_map:
			 self.load_arpabet_mapping()
		
		# update the original substituion matrix
		for key1 in arpabet_phonemes:
			for key2 in arpabet_phonemes:
				nkey1 = self.arpabet_map[key1]
				nkey2 = self.arpabet_map[key2]
				substitute_costs[ord(nkey1),ord(nkey2)] = df[key1][key2]

		result=lev(word1.encode(),word2.encode(),substitute_costs=substitute_costs)
		print(result)
		
		#normalize the levenshtein score by taking max(str1,str2)
		denominator = max(len(word1),len(word2))
		normalized_score = result / float(denominator)
		print(normalized_score)
		return normalized_score

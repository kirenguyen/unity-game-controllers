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
		self.arpabet_map = dict()
	

	def process_Speechace_results(self,results):
		'''
		takes in results from speechace and analyze the user's pronunciation accuracy
		'''
		self.iword = results["word"].encode("ascii","ignore")
		self.load_arpabet_mapping()

		self.pho_results = results["phone_score_list"]
		self.syl_results = results["syllable_score_list"]

		phoneme_acc = self.phoneme_based_accuracy(self.pho_results)
		syllable_acc = self.syllable_based_accuracy(self.syl_results)


		return self.obtain_pronunciation_message_for_ROS(phoneme_acc, syllable_acc)


	def phoneme_based_accuracy(self,phoneme_results):
		''''
		checks which graphemes pass the accuracy threshold
		return: a tuple that contains three lists (graphemes,phonemes and bool values indicating whether the graphemes pass threshold)
		'''
		
		graphemes,phonemes = self.phonemes2graphemes(self.iword)
		pass_list = []

		for i in range(len(phonemes)):
			score = self.pho_results[i]["quality_score"]
			if score >= SCORE_THRESHOLD:
				pass_list.append(True)
			else:
				pass_list.append(False)

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
				pass_list.append(True)
			else:
				pass_list.append(False)

		return (syllable_list, pass_list)

	def obtain_pronunciation_message_for_ROS(self, phoneme_acc, syllable_acc):
		'''
		should be called by iSpy Game Controller to get the final pronunciation message for ROS
		return: a dictionary of letters with bool values indicating whether letters should be red or greeen
		'''
		
		# Uncomment for phoneme accuracy
		speech_result = {}
		index = 0
		graphemes, phonemes, scores = phoneme_acc

		for i in range(len(graphemes)):
			if len(graphemes[i]) > 1:
				for j in graphemes[i]:
					speech_result["%s-%s"%(j,index)] = scores[i]
					index += 1
			else:
				speech_result["%s-%s"%(graphemes[i],index)] = scores[i]
				index += 1


		# #Uncomment for syllable accuracy
		# speech_result = {}
		# index = 0
		# syllables, scores = syllable_acc
		# for i in range(len(syllables)):
		# 	if len(syllables[i]) > 1:
		# 		for j in syllables[i]:
		# 			speech_result["%s-%s"%(j,index)] = scores[i]
		# 			index += 1
		# 	else:
		# 		#Changed graphemes to syllables
		# 		speech_result["%s-%s"%(syllables[i],index)] = scores[i]
		# 		index += 1


		# print (speech_result)
		return speech_result



	def load_arpabet_mapping(self):

		dir_path = os.path.dirname(os.path.realpath(__file__))

		# dictionary to store arpabet mapping 
		
		with open(dir_path + '/arpabet_mapping.csv','r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				#row[1] is CMU arpabet, row[0] is a modified version of arpabet for NETTalk database
 				self.arpabet_map.update({row[1]:row[0]})

 		#load nettalk.data dictionary, which contains alignment of graphemes and phonemes
		self.load_nettalk_dataset()


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


	def phonemes2graphemes(self,iword):
		'''
		align a given word's graphemes with its phonemes
		'''
		import pronouncing
		phonemes_raw=pronouncing.phones_for_word(iword)[0].split(' ')
		phonemes=[''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
		
		#find the phoneme-grapheme alignment in Nettalk database
		#get exact alignment
		phos=self.nettalk_dataset[iword]
		
		# print(phos)
	
		phos=list(phos)
		word_list = list(iword)
		graphemes = list()
		grapheme=''
		for i in range(0,len(phos),1):
			if phos[i] != '-':
				# one phoneme is matched to one letter
				if grapheme!='':
					graphemes.append(grapheme)
				grapheme = iword[i]
			else:
				# one phoneme is matched to an additional letter
				grapheme+=iword[i]
		graphemes.append(grapheme)

		# For some reason camera doesn't line up phonemes and graphemes correctly
		if iword == "camera":
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
		import pronouncing
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
		#/anaconda3/lib/python3.6/site-packages/weighted_levenshtein
		# delete "from clev import *" in __init__.py

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
			print("load mapping")
			self.load_arpabet_mapping()
			print(self.arpabet_map)
		
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


#test = PronunciationResultsHandler()
#out=test.measure_weighted_levenshtein_distance("tiger","pig")


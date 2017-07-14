import pronouncing
import csv
import os

SCORE_THRESHOLD = 70

class PronunciationHandler:
	'''
	This class contains functions that handles pronunciation results obtained from SpeechACE
	'''

	def __init__(self):

		#loads arpabet mapping, an IPA-like representation of phonemes suited for computer processing
		self.load_arpabet_mapping()

		#load nettalk.data dictionary, which is used to alignment word graphemes and phonemes
		self.load_nettalk_dataset()
	

	def process_speechace_word_results(self,word_results):
		'''
		takes in results from speechace and analyze the user's pronunciation accuracy
		'''
		self.current_word = word_results["word"]#.encode("ascii","ignore")

		self.pho_results = word_results["phone_score_list"]
		self.syl_results = word_results["syllable_score_list"]

		phoneme_acc = self.phoneme_based_accuracy(self.pho_results)
		syllable_acc = self.syllable_based_accuracy(self.syl_results)

		return self.get_letters_and_scores(phoneme_acc, syllable_acc)


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
				pass_list.append("1")
			else:
				pass_list.append("0")

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

	def get_letters_and_scores(self, phoneme_acc, syllable_acc):
		'''
		called by  Game Controller to get the letters and whether they were passing or not
		'''
		
		# Uncomment for phoneme accuracy
		speech_result = {}
		index = 0
		graphemes, phonemes, scores = phoneme_acc
		letters = []
		passed = []

		for i in range(len(graphemes)):
			if len(graphemes[i]) > 1: #some graphemes have multiple letters, like 'CH'
				for j in graphemes[i]:
					letters.append(j)
					passed.append(scores[i])

			else:
				letters.append(graphemes[i])
				passed.append(scores[i])


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

		return letters, passed
		#return speech_result



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
		print(word)
		print(pronouncing.phones_for_word(word.lower()))

		#need to use lowercase form of word for pronouncing and nettalk dict lookup
		phonemes_raw = pronouncing.phones_for_word(word.lower())[0].split(' ')
		phonemes=[''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
		
		#find the phoneme-grapheme alignment in Nettalk database
		#get exact alignment
		phos=self.nettalk_dataset[word.lower()]
		
		print(phos)
	
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

		# TODO: This is hack-ey and should not be done. Let this comment stand as a reminder
		# TODO: that sometimes phonemes and graphemes dont line up right...

		# For some reason camera doesn't line up phonemes and graphemes correctly
		#if word == "camera":
		#	graphemes = ['c', 'a', 'm', 'er', 'a']

		print([graphemes,phonemes])
		return [graphemes,phonemes]

	#def convert_arpabet_to_unique_ids(self):
	# 	'''
	# 	maps phonemes from CMU arpabet to unique index numbers 
	# 	'''
	# 	self.arpabet_ids = dict()
		
	def conversion_for_phonetic_similarity(self, word):
		'''
		convert a given word into a unique phonetic transcription, 
		which allows for measuring phonetic siimlarity with other words
		output: a phonetic string for the input word. each letter in the string corresponds uniquely to a phone
		'''
		phonemes_raw=pronouncing.phones_for_word(word)[0].split(' ')
		phonemes=[''.join(filter(lambda c: not c.isdigit(), pho)) for pho in phonemes_raw]
		print(phonemes_raw)
		output = ''
		for phoneme in phonemes:
			if phoneme in self.arpabet_map:
				output+=self.arpabet_map[phoneme]
			else:
				print("phone ( "+phoneme+" ) does not exist in arpabet map")
				break
		print(output)
		return output


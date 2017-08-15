#!/usr/bin/env python

import binascii
import json
import subprocess
import time
import wave

import pyaudio
import rospy
import _thread as thread
from r1d1_msgs.msg import AndroidAudio
from six.moves import queue

from GameUtils import GlobalSettings


class iSpyAudioRecorder:
	#CONSTANTS
	FORMAT = pyaudio.paInt16
	CHANNELS = 1
	RATE = 16000
	CHUNK = 16000
	RECORD_SECONDS = 4
	WAVE_OUTPUT_FILENAME = "audioFile.wav"

	EXTERNAL_MIC_NAME = 'USB audio CODEC: Audio (hw:1,0)'

	ROS_TO_ANDROID_MIC_TOPIC = 'android_audio'

	def __init__(self):
		#True if the phone is recording
		self.isRecording = False
		
		#Holds the audio data that turns into a wave file
		#Needed so that the data will be saved when recording audio in the new thread
		self.buffered_audio_data = []

		#0 if never recorded before, odd if is recording, even if finished recording	
		self.has_recorded = 0

	def _audio_data_generator(self, buff, buffered_audio_data):
		"""Takes the buffer and adds it to the list data. 
		Stops recording after self.isRecording becomes False
		"""

		while True:
			if self.isRecording == False:
				self.sub_audio.unregister()	#Unregisters from topic when not recording
				break

			buffered_audio_data += [buff.get()]

		return buffered_audio_data
		

	def _fill_buffer(self, audio_stream, args):
		"""Converts the microphone data from hex to binascii
		"""
		buff = args
		buff.put(binascii.unhexlify(audio_stream.data))
		

	def record_android_audio(self, buffered_audio_data):
		"""Creates a queue that will hold the audio data
		Subscribes to the microphone to receive data
		Returns the buffered audio data
		"""
		buff = queue.Queue()
		self.sub_audio = None
		self.sub_audio = rospy.Subscriber(self.ROS_TO_ANDROID_MIC_TOPIC, AndroidAudio, self._fill_buffer, buff)

		return self._audio_data_generator(buff,buffered_audio_data)

	def record_usb_audio(self, buffered_audio_data):
		"""Opens pyaudio and finds which microphone to use. Then records from that
		device until self.isRecording is False. 
		"""
		mic_index = None
		audio = pyaudio.PyAudio()
		info = audio.get_host_api_info_by_index(0)
		numdevices = info.get('deviceCount')
		print(numdevices)

		for i in range(0, numdevices):
			if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
				print(audio.get_device_info_by_host_api_device_index(0, i).get('name'))
				if audio.get_device_info_by_host_api_device_index(0, i).get('name') == self.EXTERNAL_MIC_NAME:
					mic_index = i
					break

		if mic_index == None:
			print('NOT RECORDING, NO USB AUDIO DEVICE FOUND!')
			pass
		else:
			# start Recording
			print('USB Audio Device found, recording!')
			stream = audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE, input=True, frames_per_buffer=self.CHUNK, input_device_index=mic_index)

			# TODO Sometimes there will be an error that there was no audio data or it doesnt record everything
			# Find out the right CHUNK size or delay so that all of the audio is correctly recorded
			while self.isRecording:
				data = stream.read(self.CHUNK)
				buffered_audio_data.append(data)

			# Stops the recording
			stream.stop_stream()
			stream.close()
			audio.terminate()

	def speechace(self, audio_file, correct_text):
		"""Takes the audiofile and the text that it is supposed to match and returns a 
		score and the time elapsed to calculate the score.
		"""
		start_time = time.time()

		# send request to speechace api
		api_command="curl --form text='"+correct_text+"' --form user_audio_file=@"+audio_file+" --form dialect=general_american --form user_id=1234 \"https://api.speechace.co/api/scoring/text/v0.1/json?key=po%2Fc4gm%2Bp4KIrcoofC5QoiFHR2BTrgfUdkozmpzHFuP%2BEuoCI1sSoDFoYOCtaxj8N6Y%2BXxYpVqtvj1EeYqmXYSp%2BfgNfgoSr5urt6%2FPQzAQwieDDzlqZhWO2qFqYKslE&user_id=002\"";
		p = subprocess.Popen(api_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		pouts=p.stdout.readlines()
		out_json=pouts[3]

		elapsed_time = time.time() - start_time

		# decode json outputs from speechace api

		try:
			result= json.loads(out_json)['text_score']
			result_word_score_list=result['word_score_list']
		
			return result_word_score_list
		except:
			print("DID NOT GET A VALID RESPONSE")
			return None

	def speakingStage(self, string):
		"""Takes a string and if it is speakingStart it starts a new thread that records
		the microphones audio.
		If the string is speakingEnd, it ends the recording and makes the data into
		a wave file
		"""
		if string == "speakingStart":
			self.isRecording = True
			self.has_recorded += 1
			self.buffered_audio_data = []	#Resets audio data

			if GlobalSettings.USE_USB_MIC:
				thread.start_new_thread(self.record_usb_audio, (self.buffered_audio_data,))
				time.sleep(.1)
			else:
				thread.start_new_thread (self.record_android_audio, (self.buffered_audio_data, ))
				time.sleep(.1)

		elif string == "speakingEnd":
			self.isRecording = False 		#Ends the recording
			self.has_recorded += 1
			# TODO Check if changing this time has any effect on errors in recording with USB
			time.sleep(.2)				#Gives time to return the data

			waveFile = wave.open(self.WAVE_OUTPUT_FILENAME, 'wb')
			waveFile.setnchannels(self.CHANNELS)
			waveFile.setsampwidth(2)
			waveFile.setframerate(self.RATE)
			waveFile.writeframes(b''.join(self.buffered_audio_data))
			waveFile.close()

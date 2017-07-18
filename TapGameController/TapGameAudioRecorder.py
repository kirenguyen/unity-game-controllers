#!/usr/bin/env python


import json
import pyaudio
import _thread as thread
import binascii
import wave
import time
import subprocess
from six.moves import queue

from transitions import Machine
from .TapGameUtils import GlobalSettings
from .TapGameUtils import Utils
from .StudentModel import StudentModel


if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from r1d1_msgs.msg import AndroidAudio
else:
    TapGameLog = GlobalSettings.TapGameLog #Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand





#import TapGameFSM


class TapGameAudioRecorder:

    # CONSTANTS
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = 4
    WAV_OUTPUT_FILENAME = "audioFile.wav"
    ANDROID_MIC_TO_ROS_TOPIC = 'android_audio'


    def __init__(self):
        # True if the phone is recording
        self.isRecording = False

        # Holds the audio data that turns into a wave file
        # Needed so that the data will be saved when recording audio in the new thread
        self.buffered_audio_data = []

        # 0 if never recorded before, odd if is recording, even if finished recording
        self.has_recorded = 0

        

    def _audio_data_generator(self, buff, data):
        """Takes the buffer and adds it to the list data.
        Stops recording after self.isRecording becomes False
        """

        while True:
            if self.isRecording == False:
                self.sub_audio.unregister()  # Unregisters from topic when not recording
                break

            data += [buff.get()]

        return data

    def _fill_buffer(self, audio_stream, args):
        """Converts the microphone data from hex to binascii
        """
        buff = args
        buff.put(binascii.unhexlify(audio_stream.data))

    def record_audio(self, data):
        """Creates a queue that will hold the audio data
        Subscribes to the microphone to receive data
        Returns the buffered audio data
        """
        buff = queue.Queue()
        self.sub_audio = None
        self.sub_audio = rospy.Subscriber(TapGameAudioRecorder.ANDROID_MIC_TO_ROS_TOPIC, AndroidAudio, self._fill_buffer,
                                          buff)

        return self._audio_data_generator(buff, data)

    def speechace(self, audio_file, correct_text):
        """Takes the audiofile and the text that it is supposed to match and returns a
        score and the time elapsed to calculate the score.
        """
        start_time = time.time()

        # send request to speechace api
        api_command = "curl --form text='" + correct_text + "' --form user_audio_file=@" + audio_file + " --form dialect=general_american --form user_id=1234 \"https://api.speechace.co/api/scoring/text/v0.1/json?key=po%2Fc4gm%2Bp4KIrcoofC5QoiFHR2BTrgfUdkozmpzHFuP%2BEuoCI1sSoDFoYOCtaxj8N6Y%2BXxYpVqtvj1EeYqmXYSp%2BfgNfgoSr5urt6%2FPQzAQwieDDzlqZhWO2qFqYKslE&user_id=002\"";
        p = subprocess.Popen(api_command, shell=True, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
        pouts = p.stdout.readlines()
        out_json = pouts[3]

        elapsed_time = time.time() - start_time

        # decode json outputs from speechace api
        try:
            result = json.loads(out_json)['text_score']
            print('result is:')
            print(result)
            result_text = result['text']
            result_qualityScore = result['quality_score']
            result_wordScoreList = result['word_score_list']

            return result_wordScoreList
        except:
            print("DID NOT GET VALID RESPONSE")
            return None


    def startRecording(self):
        """
        Starts a new thread that records the microphones audio.
        """
        self.isRecording = True
        self.has_recorded += 1
        self.buffered_audio_data = []  # Resets audio data

        #only do the recording if we are actually getting streaming audio data
        if Utils.rostopic_present(TapGameAudioRecorder.ANDROID_MIC_TO_ROS_TOPIC):           
            thread.start_new_thread(self.record_audio, (self.buffered_audio_data,))
            time.sleep(.1)

    def stopRecording(self):
        """
        ends the recording and makes the data into
        a wave file
        """
        self.isRecording = False  # Ends the recording
        self.has_recorded += 1
        time.sleep(.2)  # Gives time to return the data

        #only if we are actually getting streaming audio data
        if Utils.rostopic_present(TapGameAudioRecorder.ANDROID_MIC_TO_ROS_TOPIC):
            waveFile = wave.open(TapGameAudioRecorder.WAV_OUTPUT_FILENAME, 'wb')
            waveFile.setnchannels(TapGameAudioRecorder.CHANNELS)
            waveFile.setsampwidth(2)
            waveFile.setframerate(TapGameAudioRecorder.RATE)
            waveFile.writeframes(b''.join(self.buffered_audio_data))
            waveFile.close()

"""
This file implements the student model for the storybook controller.

The model attempts to update the belief that a student can correctly pronounce
a word, and the belief that a student knows the meaning of that word. These
beliefs are correlated but updated under different criteria.
"""

import random
import sys
import nltk

from unity_game_msgs.msg import StorybookCommand
import storybook_controller.robot_feedback as robot_feedback


"""
Set up phoneme information.
"""

# Takes about 6 seconds to load this.
# ARPABET = nltk.corpus.cmudict.dict()

# # These are the 39 phonemes we should expect to see.
# # Doesn't include stress, which is 0, 1 or 2 immediately following a phoneme.
# ARPABET_PHONEMES = [
#   "aa", "ae", "ah", "ao", "aw", "ay", "b", "ch", "d", "dh", "eh", "er", "ey",
#   "f", "g", "hh", "ih", "iy", "jh", "k", "l", "m", "n", "ng", "ow", "oy", "p",
#   "r", "s", "sh", "t", "th", "uh", "uw", "v", "w", "y", "z", "zh"
# ]

# print("Loaded arpabet phonemes dict!")


"""
Load stop words.
"""
STOP_WORDS = nltk.corpus.stopwords.words("english")

"""
Updates model with inputs from SpeechACE and the child's answers to questions.
Can be queried for appropriate questions Jibo should ask the child.
"""
class StudentModel(object):
  def __init__(self):
    # Threshold above which we don't consider the word as a priority for improving,
    # meaning that it is likely not to be asked about by Jibo.
    self.GOOD_PRONOUNCE_THRESHOLD = 90.0 # Out of 100

    # Higher decay increases how much having already asked a word
    # affects the chance of asking it again.
    self.REPEAT_DECAY_WEIGHT = 5.0 # Out of 100
    
    self.word_pronunciation_scores = {}
    self.phoneme_scores = {}
    self.word_tap_scores = {} # Map word to array of ints where 1 means correct, 0 incorrect.
    self.scene_object_tap_scores = {} # Map label to array of ints similar to above.
    self.confused_word_pairs = [] # Array of tuples of expected word and child's (wrong) word.
    self.confused_label_pairs = [] # Array of tuples of expected label and child's label.

    # Keep track of trend of overall scores per page.
    self.overall_quality_scores = []

    # Current questions we've told the controller about for this page.
    self.current_questions = []
    self.on_page_asked_words = []
    self.on_page_asked_labels = []

    self.open_ended_prompts = {}

    # Current sentences on the page in evaluate mode.
    self.current_sentences = []
    self.sentences_by_page_number = {}
    self.current_scene_objects = []

    # Target words.
    self.target_words = ["donned", "spatula", "unruly", "zig-zag", "bunting",
                          "helium", "donkey", "raft", "siren", "miserable", "kite", "plain",
                          "jet", "fog", "cab", "jog"]

    # TODO: remove when not hardcoding anymore.
    # For hardcoding Henry's Happy Birthday questions.
    self.henry_questions = {}
    self.setup_henry_questions()
    self.clifford_questions = {}
    self.setup_clifford_questions()

  def update_with_duration(self, duration, text):
    """
    Update model given that the given text took <duration> seconds for
    the child to speak.
    """
    pass

  def update_with_speechace_result(self, speechace_result):
    """
    Updates the model given the result of a recent speechace analysis.
    """
    print("Updating with speech ace results!")
    if "text_score" not in speechace_result:
      print("--- Got speech ace results but no text score, likely speech was empty")
      return
    text_score = speechace_result["text_score"]
    # Record overall quality score.
    self.overall_quality_scores.append(text_score["quality_score"])
    # Record scores for each word.
    for word_score in text_score["word_score_list"]:
      # Convert all words to lowercase.
      word = self.strip_punctuation(word_score["word"].lower())
      if word not in self.word_pronunciation_scores:
        self.word_pronunciation_scores[word] = []
      self.word_pronunciation_scores[word].append(word_score["quality_score"])
      # Record scores for each phoneme of each word.
      for phoneme_score in word_score["phone_score_list"]:
        phoneme = phoneme_score["phone"]
        if phoneme not in self.phoneme_scores:
          self.phoneme_scores[phoneme] = []
        self.phoneme_scores[phoneme].append(phoneme_score["quality_score"])


    self.plot_distribution()

    # TODO something more sophisticated given this score information?

  def update_with_correct_word_tapped(self, word):
    """
    Update model given that the child tapped on a word correctly.
    """
    if word not in self.word_tap_scores:
      self.word_tap_scores[word] = []
    self.word_tap_scores[word].append(1)

  def update_with_incorrect_word_tapped(self, expected_word, word):
    """
    Update model given that the child tapped on <word> when asked to tap on
    <expected_word>.
    """
    if word not in self.word_tap_scores:
      self.word_tap_scores[word] = []
    self.word_tap_scores[word].append(0)
    self.confused_word_pairs.append((expected_word, word))

  def update_with_correct_scene_object_tapped(self, label):
    """
    Update model given that the child tapped on a scene object correctly.
    """
    if label not in self.scene_object_tap_scores:
      self.scene_object_tap_scores[label] = []
    self.scene_object_tap_scores[label].append(1)


  def update_with_incorrect_scene_object_tapped(self, expected_label, label):
    """
    Update model given that the child tapped on a scene object with label <label>
    when asked to tap on an object with label <expected_label>
    """
    if label not in self.scene_object_tap_scores:
      self.scene_object_tap_scores[label] = []
    self.scene_object_tap_scores[label].append(0)
    self.confused_label_pairs.append((expected_label, label))

  def update_with_correct_word_pronounced(self, word):
    """
    Update model given that the child was shown the text of a word and they
    pronounced it correctly.
    """
    pass

  def update_with_incorrect_word_pronounced(self, word):
    """
    Update model given that the child was shown the text of a word and they
    pronounced it incorrectly.
    """
    pass

  def update_with_explore_word_tapped(self, word):
    """
    Update model given that child tapped on a word while in explore mode.
    """
    pass

  def update_with_explore_scene_object_tapped(self, label):
    """
    Update model given that child tapped on a scene object with the given label
    while in explore mode.
    """
    pass

  def is_child_turn(self, sentence_index):
    """
    In a turn taking exercise, returns whether or not the child should be
    asked to read the sentence at the given index.
    """
    return True

  def update_current_page(self, story, page_num, sentences, scene_objects, prompts):
    self.page_num = page_num
    self.story = story
    self.current_questions = []
    self.on_page_asked_word = []
    self.on_page_asked_labels = []
    self.sentences_by_page_number[page_num] = []
    for sentence in sentences:
      self.sentences_by_page_number[page_num].append(sentence.split())
    self.current_sentences = self.sentences_by_page_number[page_num]
    self.current_scene_objects = scene_objects
    self.open_ended_prompts[page_num] = []
    for p in prompts:
      self.open_ended_prompts[page_num].append(robot_feedback.EndPageQuestionOpenEndedVerbalResponse(
        p.question, p.response))

    # # Use this to hardcode questions for the robot.
    # if page_num == 3:
    #   self.henry_questions[page_num].append(robot_feedback.EndPageQuestionSceneObjectTap("spatula", self.get_scene_object_ids("spatula")))
    # elif page_num == 6:
    #   self.henry_questions[page_num].append(robot_feedback.EndPageQuestionWordTap("bunting", self.get_word_indexes("bunting")))
    # elif page_num == 20:
    #   self.henry_questions[page_num].append(robot_feedback.EndPageQuestionSceneObjectTap("bunting", self.get_scene_object_ids("bunting"), None, "This is a bunting. It's what's above the dining table!"))
    # elif page_num == 22:
    #   self.henry_questions[page_num].append(robot_feedback.EndPageQuestionWordTap("helium", self.get_word_indexes("helium")))
    # elif page_num == 25:
    #   self.henry_questions[page_num].append(robot_feedback.EndPageQuestionSceneObjectTap("zig-zag", self.get_scene_object_ids("zig-zag"), "Can you tap on the zig-zag lines in the picture? I think they're on the crocodile!"))
    # elif page_num == 26:
    #   self.henry_questions[page_num].append(robot_feedback.EndPageQuestionWordtap("siren", self.get_word_indexes("siren")))


  def get_end_page_questions(self, prev_times_asked):
    """
    If prev_times_asked is 0, it's the first time we're asking for questions on this page.
    If it's 1, then it's the second time. We only support asking up to two times, which
    gives enough time for speechace results to come in by the second time
    """

    hardcoded = {}
    hardcoded[self.page_num] = []
    if self.story == "henrys_happy_birthday":
      hardcoded = self.henry_questions
    elif self.story == "clifford_and_the_jet":
      hardcoded = self.clifford_questions
    # Preexisting questions exist, so first return those then generate new ones the second time.
    if len(hardcoded[self.page_num]) > 0:
      if prev_times_asked == 0:
        self.current_questions = hardcoded[self.page_num]
      elif prev_times_asked == 1:
        self.current_questions = hardcoded[self.page_num] + self.get_additional_end_page_questions()
    # No preexisting questions exist, so just generate new ones right now.
    else:
      if prev_times_asked == 0:
        self.current_questions = self.get_additional_end_page_questions()
      elif prev_times_asked == 1:
        self.current_questions = self.current_questions + self.get_additional_end_page_questions()

    return self.current_questions

  def get_additional_end_page_questions(self):
    """
    Called after we've asked some number of prescribed questions OR
    if there were no prescribed questions.

    Autogenerate questions based on target words, labels that appear
    in the text, speech ace scores, and overlap with prescribed
    open ended questions.

    Possible to return nothing.
    """

    print("Getting additional end page questions...")

    # 25% chance try to get scene object first, 75% try to get word first.
    word_questions = self.try_get_word_questions()
    scene_object_questions = self.try_get_scene_object_questions()
    if random.random() < .25:
      if len(scene_object_questions) > 0:
        self.on_page_asked_labels += map(lambda s: s.expected_label, scene_object_questions)
        print("added something to labels", self.on_page_asked_labels)
        return scene_object_questions
      self.on_page_asked_words += map(lambda w: w.expected_word, word_questions)
      return word_questions
    else:
      if len(word_questions) > 0:
        self.on_page_asked_words += map(lambda w: w.expected_word, word_questions)
        return word_questions
      self.on_page_asked_labels += map(lambda s: s.expected_label, scene_object_questions)
      print("added something to labels", self.on_page_asked_labels)

      return scene_object_questions
    

  def try_get_word_questions(self, force=False):
    word = self.get_word_for_question(force)
    if word is not None:
      # 80% chance pronounce, 20% chance tap on word, vs. asking to pronounce that word.
      if random.random() < .20:
        return [robot_feedback.EndPageQuestionWordTap(word,
                self.get_word_indexes(word))]
      else:
        # TODO: Consider only show one of the words, otherwise might be confusing? 
        return [robot_feedback.EndPageQuestionWordPronounce(word,
                self.get_word_indexes(word))]
    else:
      return []

  def try_get_scene_object_questions(self, force_in_text=True):
    scene_object_label = self.get_scene_object_label_for_question()
    if scene_object_label is not None:
      return [robot_feedback.EndPageQuestionSceneObjectTap(scene_object_label,
              self.get_scene_object_ids(scene_object_label))]
    else:
      return []

  def get_word_for_question(self, force=False):
    """
    Return a word from the current page that we could ask a question about.

    Try to use words that are target words and that have been unseen.
    """
    on_page_target_words = []
    on_page_words = []
    for sentence in self.current_sentences:
      for w in sentence:
        formatted_w = self.strip_punctuation(w.lower())
        if formatted_w in self.on_page_asked_words:
          continue
        if formatted_w in self.target_words:
          on_page_target_words.append(formatted_w)
        on_page_words.append(formatted_w)
    print("on page target words: ", on_page_target_words, on_page_words)
    if len(on_page_target_words) > 0:
      return self.get_lowest_pronunciation_score_word(on_page_target_words)

    if not force:
      return None

    return self.get_lowest_pronunciation_score_word(on_page_words)


  def get_lowest_pronunciation_score_word(self, words):
    word_to_return = None
    no_score_words = []
    lowest_score = 100.0
    for word in words:
      if word in self.word_pronunciation_scores:
        print("scores: ", self.word_pronunciation_scores[word])
        avg_score = sum(self.word_pronunciation_scores[word]) * 1.0 / len(self.word_pronunciation_scores[word])
        # Don't include words we've asked about recently.
        # Based on heuristics. Make score higher (less likely to ask) the
        # more we've already asked it.
        weighted_score = avg_score + self.prev_times_asked_word(word) * self.REPEAT_DECAY_WEIGHT
        if avg_score < lowest_score:
          lowest_score = avg_score
          word_to_return = word
      else:
        no_score_words.append(word)
    if word_to_return is None and len(no_score_words) > 0:
      print("picking random word of no_score_words", no_score_words)
      word_to_return = no_score_words[random.randint(0, len(no_score_words) - 1)]
    if word_to_return is not None:
      return word_to_return

  def get_scene_object_label_for_question(self, force_in_text=True):
    """
    Returns a label from the current scene objects on the page.

    If force_in_text is True, can ONLY return labels that appear in the text.
    """

    # Give priority to words the child didn't get right before, and words
    # we haven't asked the child to try yet. Or words the child confused before?
    lowest_score = 1
    label_to_return = None
    not_asked = []
    all_valid_labels = []
    for scene_object in self.current_scene_objects:
      if scene_object.in_text or not force_in_text:
        label = scene_object.label
        label = " ".join(map(lambda w: self.strip_punctuation(w.lower()), label.split(" ")))
        print("Asked already", self.on_page_asked_labels)
        if label in self.on_page_asked_labels:
          continue
        if label in self.scene_object_tap_scores:
          # Percentage of correct responses.
          score = sum(self.scene_object_tap_scores[label]) * 1.0 / len(self.scene_object_tap_scores[label])
          if score < lowest_score:
            lowest_score = score
            label_to_return = label
        else:
          not_asked.append(label)
        all_valid_labels.append(label)
    if label_to_return is not None:
      print("Found label with low score:", label_to_return, lowest_score)
      return label_to_return
    else:
      # If there are no labels at all.
      if len(all_valid_labels) == 0:
        return None 
      # If there are no labels that we haven't asked about, return a random
      # label from the valid set.
      if len(not_asked) == 0:
        random_label = all_valid_labels[random.randint(0, len(all_valid_labels) - 1)]
        print("Using random label:", random_label)
        return random_label
      # Otherwise, pick a label that we haven't asked about.
      else:
        # Note that randint is inclusive on both endpoints.
        not_asked_label = not_asked[random.randint(0, len(not_asked)-1)]
        print("Using a label that we haven't used before:", not_asked_label)
        return not_asked_label

  """"
  Helpers
  """

  def get_word_indexes(self, word):
    # Go through all sentences and find the words that match.
    indexes = []
    i = 0
    for sentence in self.current_sentences:
      for w in sentence:
        if self.strip_punctuation(w.lower()) == word:
          indexes.append(i)
        i += 1
    print("get indexes", self.current_sentences, word, indexes)
    return indexes

  def get_scene_object_ids(self, label):
    ids = []
    stripped_label = self.strip_punctuation(label.lower())
    for s in self.current_scene_objects:
      if self.strip_punctuation(s.label.lower()) == stripped_label:
        ids.append(s.id)
    print("get ids", self.current_scene_objects, label, ids)
    return ids

  def get_phonemes(self, word):
    if word not in ARPABET:
      return None
    phonemes_raw = ARPABET[word][0]
    # Remove stress and turn phonemes to lowercase.
    phonemes = ["".join(filter(lambda c: c.isalpha(), p)).lower() \
                for p in phonemes_raw]
    return phonemes

  def plot_distribution(self):
    """
    Plot the current distribution.
    """
    print(self.word_pronunciation_scores)

  def prev_times_asked_word(self, word):
    if word in self.word_tap_scores:
      return len(self.word_tap_scores[word])
    else:
      return 0

  def prev_times_asked_label(self, label):
    if label in self.scene_object_tap_scores:
      return len(self.scene_object_tap_scores[label])
    else:
      return 0

  def strip_punctuation(self, word):
    return "".join([c for c in word if c.isalnum()])


  def setup_henry_questions(self):
    # There are 29 pages in Henry story, page 1 is the first non-title page.
    # Hard code some questions.
    self.henry_questions[1] = []
    self.henry_questions[2] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Can you explain the word <break size='0.2'/> <duration stretch='1.3'> donned </duration> <break size='0.2'/> to me? The book says Henry donned his slippers.",
        "Good thought. <duration stretch='1.2'> Donned </duration> means to put something on. <break size='.5'/> When it's cold, I don my favorite hat!")]
    self.henry_questions[3] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("<style set='confused'>What is a <duration stretch='1.2'> spatula </duration>?</style>",
        "So, I think a spatula is used in cooking. Henry's mom can use it to scoop, mix, or spread things while making the cake!")]
    self.henry_questions[4] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What kind of cake did Henry want?",
        "I think Henry wants chocolate cake! But his mom says it should be vanilla. <break size='.5'/> Vanilla is also good I guess."),
                                robot_feedback.EndPageQuestionOpenEndedVerbalResponse("<style set='confused'> <duration stretch='1.2'> What does plain <break size='.2'/> mean? </duration> </style>",
                                "I think <duration stretch='1.1'> plain </duration> <break size='.3'/> means boring or simple. Henry thinks vanilla cake is plain.")]
    self.henry_questions[5] = []
    self.henry_questions[6] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Do you know what <break size='.2'/> <duration stretch='1.3'> bunting </duration> <break size='.2'/> means?",
        "Ok. Bunting is what's above the dining table in the picture. <break size='.5'/> It's a decoration like a flag. For my birthday, I had a bunting that said 'Happy Birthday'!")]
    self.henry_questions[7] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("How do you think Henry feels right now?",
        "Yeah. Personally, I think Henry is nervous that he won't get any candy.")]
    self.henry_questions[8] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("<style set='confused'> What kind of pattern does Henry's favorite T-shirt have? </style>",
        "Henry's favorite shirt has a zig zag pattern. Zig zag is a line that goes left, right left, right, just like a lighting bolt!"),
                                robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Can you tell me what <break size='.2'/> <duration stretch='1.3'> unruly </duration> <break size='.2'/> means?",
                                  "<style set='confident'> I think <duration stretch='1.2'> unruly </duration> is when something is a little wild or out of control. </style> <break size='.5'/> I have a friend whose hair is very unruly in the morning.")]
    self.henry_questions[9] = []
    self.henry_questions[10] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What did Henry's mom want him to <duration stretch='1.3'> don? </duration>",
      "<duration stretch='1.2'> Henry donned a white shirt and a bowtie </duration>. <break size='.3'/>  Remember that don means to put on! <break size='.3'/> I don my swimsuit when I go to the beach")]
    self.henry_questions[11] = []
    self.henry_questions[12] = []
    self.henry_questions[13] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Do you think Henry liked getting a birthday kiss from Aunt Sue?",
      "Good thought. I'm not sure if Henry liked the big sticky red mark.")]
    self.henry_questions[14] = []
    self.henry_questions[15] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What do you think is in the big yellow box?",
      "I don't know what's in the box, <style set='enthusiastic'> but I'm excited to find out! </style>")]
    self.henry_questions[16] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Oh I love games, what's your favorite party game?",
      "<style set='enthusiastic'> Cool! My favorite party game is musical chairs. </style> ")]
    self.henry_questions[17] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What does Henry look like with the tail on his back?",
      "<style set='confident'> His friends say he looks like a <break size='.3'/> donkey <break size='.3'/>. A <duration stretch='1.1'> donkey </duration> is an animal like the one in the picture. </style> It's kind of like a horse. Some people use donkeys to help move heavy things on farms.")]
    self.henry_questions[18] = []
    self.henry_questions[19] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What does miserable mean?",
      "Good thought. <break size='.5'/> Miserable means very unhappy and sad. Henry feels miserable because his birthday isn't going the way he wanted."),
                                robot_feedback.EndPageQuestionOpenEndedVerbalResponse("The book says the room became very <duration stretch='1.2'> unruly. </duration> <break size='.5'/> What made the room unruly?",
                                  "I think the room was unruly because everyone was fighting for a chair, and everything got a little wild and out of order.")]
    self.henry_questions[20] = []
    self.henry_questions[21] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Why is Henry miserable about his birthday?",
      "Yup. I think Henry is having a bad birthday, because he didn't get the cake flavor he wanted and he didn't win any games, so he's sad and miserable.")]  
    self.henry_questions[22] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Do you know what <break size='.2'/> <duration stretch='1.3'> helium </duration> <break size='.2'/> is? Can you explain it to me?",
      "My teacher told me that <duration stretch='1.2'> helium </duration> is the gas that goes in balloons. <break size='.8'/> It's what makes them stay floating instead of falling down. The helium balloons Henry's dad got are very colorful!")]
    self.henry_questions[23] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("Henry's mom used a <duration stretch='1.2'> spatula </duration> to cut the cake. Do you remember what else a spatula is used for?",
      "Yeah, a <duration stretch='1.1'> spatula </duration> is used for many cooking tasks like mixing and spreading.")]
    self.henry_questions[24] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("<duration stretch='1.2'> Can you tell me what a kite </duration> is?",
        "I think a <duration stretch='1.1'> kite is a toy that flies through the air when the wind blows it </duration>. <break size='.4'/> <style set='enthusiastic'> I like to fly kites, it's very fun! </style>")]
    self.henry_questions[25] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("<style set='enthusiastic'> Wow that's a great gift. </style> <break size='.5'/> Do you know what a <break size='.2'/> <duration stretch='1.3'> raft </duration> <break size='.2'/> is?",
      "A raft is something that you can float on in the water. <break size='.5'/> <style set='enthusiastic'> I like floating on rafts at the swimming pool. </style> Henry's new raft is shaped like a crocodile.")]
    self.henry_questions[26] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What is <break size='.2'/> <duration stretch='1.3'> a siren? </duration> <break size='.2'/>",
      "I think a siren is something on a police car or fire truck that makes a loud ringing noise <break size='.2'/> to make people get out of the way. Sometimes <duration stretch='1.15'> siren </duration> noises wake me up at night.")]
    self.henry_questions[27] = []
    self.henry_questions[28] = []

  def setup_clifford_questions(self):
    self.clifford_questions[1] = []
    self.clifford_questions[2] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("<duration stretch='1.15'> Why won't Jim's jet fly? </duration>",
        "<style set='confident'> <duration stretch='1.2'> Jim will not fly because of the fog. The fog makes it hard to see. </duration> </style>")]
    self.clifford_questions[3] = []
    self.clifford_questions[4] = []
    self.clifford_questions[5] = [robot_feedback.EndPageQuestionOpenEndedVerbalResponse("What is a <break size='.3'/> <duration stretch='1.2'> cab? </duration> Can you explain it to me?",
      "My teacher told me that a <break size='.3'/> cab is like a car. It helps take you from one place to another. <break size='.6'/> Here, Clifford is acting like a cab because he's picking up the family to take them somewhere.")]
    self.clifford_questions[6] = []
    self.clifford_questions[7] = []
    self.clifford_questions[8] = []


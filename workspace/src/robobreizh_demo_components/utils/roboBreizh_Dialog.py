#!/usr/bin/env python
#-*- encoding: UTF-8 -*-
import qi
import sys
import argparse
import functools
import time


sys.path.append('./utils')
from data import Person

## Dialog class contains all variables & functions to allow Pepper to communicate with his speakers

class Dialog :

	## The constructor of the Dialog class
	# @param self The object pointer
	def __init__(self, session) :

		# Naoqi session
		self.session = session

		# ALTextToSpeech parameters
		self.alTextToSpeech = self.session.service("ALTextToSpeech")
		self.alTextToSpeech.setParameter("volume",100)
		self.alTextToSpeech.setParameter("pitch",100)
		self.alTextToSpeech.setParameter("speed",80)

		# ALMemory callback (to get the variable)
		self.alMemory = self.session.service("ALMemory")
		self.tagSub = self.alMemory.subscriber("Dialog/Tag")
		self.tagSub.signal.connect(functools.partial(self.stop_callback, "Dialog/Tag"))
		self.nameSub = self.alMemory.subscriber("name")
		self.nameSub.signal.connect(functools.partial(self.name_callback, "name"))
		self.drinkSub = self.alMemory.subscriber("drink")
		self.drinkSub.signal.connect(functools.partial(self.drink_callback, "drink"))

		# ALDialog
		self.alDialog = self.session.service("ALDialog")
		self.alDialog.setLanguage("English")
		self.configuration = {"bodyLanguageMode":"contextual"}
		self.alDialog.setAnimatedSpeechConfiguration(self.configuration)
		self.end_dialog = False

		# ALAnimatedSpeech
		self.alAnimatedSpeech = self.session.service("ALAnimatedSpeech")

		# Dialog attribute
		self.person = None
		self.topic_name = None
		topics = self.alDialog.getAllLoadedTopics()
		for top in topics :
			self.alDialog.unloadTopic(top)
		self.load_dialog()

		# Remove autonomous mouvement
		# self.alBackgroundMovement = self.session.service("ALBackgroundMovement")
		# self.alBackgroundMovement.setEnabled(False)

		# Remove autonomous leds
		self.alAutonomousBlinking = self.session.service("ALAutonomousBlinking")
		self.alAutonomousBlinking.setEnabled(False)

		# AJOUT SPRINT 2/3 (PRI-ROBOCUP-S9A-2020)
		# self.ansSub = self.alMemory.subscriber("Dialog/Answered")
		# self.ansSub.signal.connect(functools.partial(self.answer_callback))	
		# self.tablet = Tablet(ip)




	## The destructor of the Dialog class
	# @param self The object pointer
	def __del__(self):
		self.alDialog.deactivateTopic(self.topic_name)
		tops = alDialog.getAllLoadedTopics()
		for top in tops:
			alDialog.unloadTopic(top)
		self.session.close()



	## Pepper says the sentence that contains "tag" (refer to the self.load_dialog(self) function)
	def say(self,tag):
		self.alDialog.subscribe("my_dialog")
		self.alDialog.gotoTag(tag,self.topic_name)
		while not self.end_dialog :
		 	pass
		self.alDialog.unsubscribe('my_dialog')
		self.end_dialog=False



	## Pepper says the sentence that contains "tag" (refer to the self.load_dialog(self) function) and initialize the self.person attribute
	def say_p(self,tag,person):
		self.person=person
		self.say(tag)
		return self.person


	## Pepper says Hello + sir or madam (depending on server prediction about the customer gender. This info is available via the person parameter [person.gender]) and the sentence that contains the tag "ask_name" by calling the say_p(self,tag,person) function
	# He says: "Hello, sir/madam. I am Pepper. What can I do for you ?"
	# @param self The object pointer
	# @param person The interviewee (type: Person defined in Hotel.py)
	def ask_name(self,person):
		text = "Hi, "
		if person.gender == "M":
			text += "sir."
		elif person.gender == "F":
			text += "madam."
		self.alAnimatedSpeech.say(text,self.configuration)
		return self.say_p("ask_name",person)



	## Pepper offer to get a drink to a customer
	# He says: "Would you like a drink ?"
	# @param self The object pointer
	# @param person The interviewee (type: Person defined in Hotel.py)
	def ask_drink(self,person):
		return self.say_p("ask_drink",person)



	## Pepper describe a person
	def describe_pers(self,person):
		tex=''
		if person.gender != "" :
			text = "I have detected a "
		if person.gender == "M":
			text += "man."
		elif person.gender == "F":
			text += "madam."
		if person.distance != 0 :
			text += " The distance between me and this person is around " +str(format(person.distance, '.1f')) +' meters.'
		if person.clothes_color != "" :
			text += " The person is wearing  " +str(person.clothes_color) +' clothes.'
		if person.age != "" and str(person.age) != "(0-2)" :
			text += " The person is around  " +str(person.age) +' years old.'
		if person.skin_color != "" :
			text += " The person is  " +str(person.skin_color) +' color of skin.'

		if person.name == "" :
			text += " Is this description corresponds to you? "
		else :
			text += " Is this description corresponds to you? " +str(person.name)
		print("     -->  Pepper : " + str(text))
		self.alAnimatedSpeech.say(text,self.configuration)



	## Pepper ash the receptionnist if he can bring a specific drink to a customer
	# He says: "Could you bring him/her a {name of the drink}, please ?"
	# @param self The object pointer
	# @param person The customer (type: Person defined in Hotel.py)
	def describe_drink(self,person):
		text = "Could you bring "
		if person.gender == "M":
			text += "him "
		elif person.gender == "F":
			text += "her "
		text += "a " + person.favorite_drink + ", please?"
		self.alAnimatedSpeech.say(text,self.configuration)



	## Peper give the age of a customer to the receptionist
	# He says: "He/She is about {age of the customer} years old."
	# @param self The object pointer
	# @param person The customer (type: Person defined in Hotel.py)
	def describe_age(self,person):
		text = ""
		if person.gender == "M":
			text += "He "
		elif person.gender == "F":
			text += "She "
		text += "is about " + str(person.age) + "years old."
		self.alAnimatedSpeech.say(text,self.configuration)



	## Peper give informations about customer's top garment to the receptionist
	# He says: "He/She is wearing a {name and color of the top garment}."
	# @param self The object pointer
	# @param person The customer (type: Person defined in Hotel.py)
	def describe_cloth_top(self,person):
		text = ""
		if person.gender == "M":
			text += "He "
		elif person.gender == "F":
			text += "She "
		text += "is wearing a " + str(person.clothes[0]) + "."
		self.alAnimatedSpeech.say(text,self.configuration)



	## Peper give informations about customer's bottom garment to the receptionist
	# He says: "He/She is wearing a {name and color of the bottom garment}."
	# @param self The object pointer
	# @param person The customer (type: Person defined in Hotel.py)
	def describe_cloth_bot(self,person):
		text = ""
		if person.gender == "M":
			text += "He "
		elif person.gender == "F":
			text += "She "
		text += "is wearing a " + str(person.clothes[1]) + "."
		self.alAnimatedSpeech.say(text,self.configuration)



	## Peper give informations about customer's clothes (both top & bottom) to the receptionist
	# He says: "He/She is wearing a {name and color of the top garment} and a {name and color of the bottom garment}."
	# @param self The object pointer
	# @param person The customer (type: Person defined in Hotel.py)
	def describe_clothes(self,person):
		text = ""
		if person.gender == "M":
			text += "He "
		elif person.gender == "F":
			text += "She "
		text += "is wearing a " + str(person.clothes[0]) + " and a " + str(person.clothes[1]) + "."
		self.alAnimatedSpeech.say(text,self.configuration)	



	## Callback function whom set the self.end_dialog to True when there is no more tags in the dialog motor
	# @param self The object pointer
	# @param key Don't used but required
	# @param value A string (need to be "stop" to end the dialog)
	def stop_callback(self,key,value):
		if value=="stop" :
			self.end_dialog=True



	## Callback function whom change the self.person.name attribute (customer name)
	# @param self The object pointer
	# @param value A string (the new name of the customer)
	def name_callback(self,key,value):
		self.person.name=value


	## Callback function whom change the self.person.favorite_drint attribute (customer favorite drink)
	# @param self The object pointer
	# @param value A string (the new name of the customer favorite drink)
	def drink_callback(self,key,value):
		self.person.favorite_drink=value



	## Load qichat code in the dialog motor
	# @param self The object pointer
	def load_dialog(self):
		self.hotel = (
			'topic: ~hotel()\n'
			'language: enu\n'
			'concept:(name)  [John Mary Anna Antoine Cedric Jean-Philippe]\n'
			'concept:(drink) [Coke Wine]\n'
			'proposal: %present Hello everyone. I a robot from CNRS lab. The demonstration will begin soon. %stop \n'
			'proposal: %look_for_person Is anybody else here? %stop \n'
			'proposal: %wait_for_someone_else I am reading to welcome new guests. %stop \n'
			'proposal: %ask_name My name is Pepper. Would you like me to explain what we are doing at CROSSING? \n'
			'	u1: (yes) Sounds good to me. What is your name? \n'
			'		u2: ({My name is} _~name) Nice to meet you $1 $name=$1 %stop \n'
			'	u1: (no) You suck dude. %stop \n'
			'proposal: %say_follow_me I will guide you to a seat. Please follow me. %stop \n'
			'proposal: %look_for_chair I dont find an empty chair. %stop \n'
			'proposal: %sit You may seat here if you want. %stop \n'
			'proposal: %ask_drink Would you like a drink? \n'
			'	u1: (_~drink) I like $1 too. $drink=$1 %stop \n'
			'proposal: %ask_smthg What can I do for you? \n'
			'	u1: (taxi, {please}) Yes, sure. %stop\n'
			'proposal: %ask_taxi Hello, I would like to book a taxi. %stop \n'
			'proposal: %say_time It will arrive in ten minutes. \n'
			'	u1: ([Thanks "Thank you"] {Pepper}) Youre welcome. %stop \n'
			'proposal: %bag Do you need my services for something else? \n'
			'	u1: (bag) Ok. I will try to take the bag. If I dont take it correctly, please help me. %stop \n'
			'proposal: %end_present The demonstration is over. Thanks for you attention mate. %stop \n'
			)
		self.topic_name = self.alDialog.loadTopicContent(self.hotel)
		self.alDialog.activateTopic(self.topic_name)



if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    # Test
    dialog = Dialog(session)
    dialog.say("present")
    time.sleep(1) # Time in seconds
    p1 = Person()
    p1.distance = 2.85
    p1.clothes_color = "bleu"
    p1.age = 35-38
    p1.gender = "M"
    p1.skin_color = "white"
    p1 = dialog.ask_name(p1)
    time.sleep(1) # Time in seconds
    dialog.describe_pers(p1)
    time.sleep(5) # Time in seconds
    dialog.say("end_present")
    sys.exit(1)

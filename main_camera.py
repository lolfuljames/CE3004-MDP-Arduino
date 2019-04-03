import logging
from bt import *
from ard import *
import cv2
import pc_camera as pc
import detect_arrow as camera
import re
import threading
from time import sleep

logging.basicConfig(filename='loggingtest1.log',level=logging.DEBUG)
cap = cv2.VideoCapture("/dev/video0")
class RPI():
	def __init__(self):
		threading.Thread.__init__(self)
		self.arduinothread = arduino()
		self.btthread = androidbt()
		self.pc = pc.pc()
		self.arduinothread.init_ser()
		self.btthread.initbt()
		#self.cap = cv2.VideoCapture("/dev/video0")

	# function for Arduino
	# sending to arduino
	def sendtoarduino(self, arduinomsg):
		if (self.arduinothread.arduinoisconnected and arduinomsg):
			self.arduinothread.writeserial(arduinomsg)
			logging.info("Message send to Arduino: %s" % arduinomsg)
			return True
#		print("fail to send")
		return False

	# receiving from arduino
	def receivefromarduino(self):
		while True:
			sermsg = self.arduinothread.readserial()
#			print("receive from ard")
#			print(sermsg)
			sermsg = sermsg.decode('utf-8')
#			print(sermsg[0])

			if (self.arduinothread.arduinoisconnected and sermsg):
#				check if RPi is ready to read
#				if(sermsg[0].lower() == 'r'):
				print('Message received from Arduino: %s' %sermsg)
#				input("CONTINUE TO SEND TO ALGO?\n")
				#send message to PC
				if(sermsg[0].lower() == 'a'):
#					print("Sending '%s' to Algo" %sermsg[1:])
					self.pc.receive_information(sermsg)
					logging.info("sent '%s' to Algo" %sermsg[1:])

				#send message to Arduino
				elif(sermsg[0].lower() == 'b'):
#					print("Sending '%s' to Android" %sermsg[1:])
					sersent = self.sendtoandroid(btmsg[1:])
					#serreply = self.sendtoandroid("Send Completed \n")
#					print("")

				else:
					print("Incorrect device selected from Arduino: %s" %(sermsg[1:]))

	#function for Android
	# sending to android
	def sendtoandroid(self,androidmsg):
		if (self.btthread.bt_connected() and androidmsg):
			self.btthread.writebt(androidmsg)
			logging.info("Sending message to Android: %s" %androidmsg)
			return True
		return false

	# receiving from android
	def receivefromandroid(self):
		while True:
			btmsg = self.btthread.readbt2()

			if (self.btthread.btconnected and btmsg):
				#check if RPi is ready to read
				if(str.lower(btmsg[0]) == 'r'):
#					print('Message received from Android: %s' %btmsg)
					btsend = self.sendtoarduino(btmsg[1:])
#					print('sent success')

				#send message to PC
				elif(str.lower(btmsg[0]) == 'a'):
#					print("Sending '%s' to Algo" %btmsg[1:])
#					input("continue to send to algo")
					self.pc.receive_information(btmsg)
					logging.info("Sent '%s' to Algo" %btmsg)

				#send message to Arduino
				elif(str.lower(btmsg[0]) == 's'):
#					print("Sending '%s' to Arduino" %btmsg[1:])
					btsend = self.sendtoarduino(btmsg[1:])
					logging.info(btsend)
					logging.info("SENT ABOVE")
					#btreply = self.sendtoarduino("Send Completed \n")
#					print("")

				else:
					logging.log("Incorrect device selected from Android: %s" %(btmsg[1:]))

	# pc function
	# sending to pc
	def send_to_pc(self, pcmsg):
		if (self.pc and pcmsg):
			self.pc.receive_information(pcmsg)
			logging.info("sending message to pc: %s" %pcmsg)
			return True
		return False

	def receive_from_pc(self):
		while True:
			pcmsg = self.pc.send_information()
			logging.info(pcmsg)
			if (self.pc and pcmsg):
				if (isinstance(pcmsg, list)):
#					print("sending movement %s to audrino" %pcmsg[0])
					self.sendtoarduino(pcmsg[0])
#					print("sent movement to audrino")
#					print("sending mdf string %s to andriod" %pcmsg[1])
					self.sendtoandroid(pcmsg[1])
#					print("sent mdf string to andriod")

				elif (isinstance(pcmsg, str)):
#					print("sending string %s to both andriod and audrino" %pcmsg)
					self.sendtoandroid(pcmsg)
					self.sendtoarduino(pcmsg)
#					print("sent to both")

				else:
					print("incorrect message obtained")

	def receive_from_camera(self):
		time1 = time.time()
		sent = False
		while True:
#			ret, frame = cap.read()
#			frame = cv2.flip(frame,1)
#			cv2.imshow("POV", frame)
#			if(cv2.waitKey(1)):
#				continue
                        position = camera.get_arrow(cap)
			if(position != 9):
				if(time.time() - time1 >= 0.3):
					if(position == 0):
						self.pc.set_arrow("B")
					if(position == 1):
						self.pc.set_arrow("M")
					else:
						self.pc.set_arrow("F")
					time1 = time.time()

	def start_thread(self):
		# Android read and write thread
		read_android_thread = threading.Thread(target = self.receivefromandroid, args = (), name = "read_android_thread")
#		write_android_thread = threading.Thread(target = self.sendtoandroid, args = (), name = "write_android_thread")

		# Arduino read and write thread
		read_arduino_thread = threading.Thread(target = self.receivefromarduino, args = (), name = "read_arduino_thread")
#		write_arduino_thread = threading.Thread(target = self.sendtoarduino, args = (), name = "write_arduino_thread")

		# pc thread
		read_pc_thread = threading.Thread(target = self.receive_from_pc, args = (), name = "read_pc_thread")
#		write_pc_thread = threading.Thread(target = self.send_to_pc, args = (), name = "write_pc_thread")

		# camera thread
		read_camera_thread = threading.Thread(target = self.receive_from_camera, args = (), name = "read_camera_thread")		
		read_android_thread.daemon = True
#		write_android_thread.daemon = True

		read_arduino_thread.daemon = True
#		write_arduino_thread.daemon = True

		read_pc_thread.daemon = True
#		write_pc_thread.daemon = True

		read_camera_thread.daemon = True

		read_android_thread.start()
#		write_android_thread.start()

		read_arduino_thread.start()
#		write_arduino_thread.start()

		read_pc_thread.start()
#		write_pc_thread.start()

		read_camera_thread.start()

	def close_all(self):
		self.arduinothread.closeserial()
		self.btthread.closebt()

if __name__ == "__main__":
	logging.info("\n\n\n------------------starting main script----------------")
	main = RPI()
	try:

		main.start_thread()
		while True:
			sleep(1)			
	except KeyboardInterrupt:
		print("Exiting the program")
		main.close_all()

import numpy as np
import time as time
from ShortestPathRB import ShortestPathRB
from map_variable import *
import logging
logging.basicConfig(filename='loggingtest1.log',level=logging.DEBUG,format='%(asctime)s %(levelname)s\n%(message)s', datefmt='%H:%M:%S')
class Exploration:
	#make sure initialize methods only run once
	def run_once(f):
		def wrapper(*args, **kwargs):
			if not wrapper.has_run:
				wrapper.has_run = True
				return f(*args, **kwargs)
		wrapper.has_run = False
		return wrapper
	#initialize robot with starting position and direction
	def __init__(self, robotStart,direction,rpi=None):
		self.phase = 1
		self.timeLim = 6
		self.explored = 0
		self.mdf1 = ''
		self.mdf2 = ''
		self.previousMvmt = FRWD
		self.update = True
		self.backtrack = False
		self.calib = 0
		self.goto = []
		self.path = []
		self.current = np.asarray(robotStart)
		self.direction = direction
		self.ghost=np.zeros([MAX_ROW,MAX_COL],dtype=int)
		self.visited=np.zeros([MAX_ROW,MAX_COL],dtype=int)#how many times robot visited the coordinates
		self.currentMap = np.zeros([MAX_ROW,MAX_COL],dtype=int)#0=unexplored, 1 = empty, 2 = obstacles
		self.updateRobotPosition()#set robot initial position to explored
		self.rpi = rpi    #actual or SIM
	def getNeighbors(self,coordinates):
		neighbors = np.asarray([[coordinates[0]-1,coordinates[1]-1],
		[coordinates[0]-1,coordinates[1]],
		[coordinates[0]-1,coordinates[1]+1],
		[coordinates[0],coordinates[1]-1],
		[coordinates[0],coordinates[1]],
		[coordinates[0],coordinates[1]+1],
		[coordinates[0]+1,coordinates[1]-1],
		[coordinates[0]+1,coordinates[1]],
		[coordinates[0]+1,coordinates[1]+1]
		])
		return neighbors
	#update unexplored map based on current position
	def updateRobotPosition(self):
		robotPos = self.current
		neighbors = self.getNeighbors(robotPos)
		for n in neighbors:
			self.setExplored(n)
			#to prevent ghost blocks
			self.ghost[n[0]][n[1]]+=1
		#set visited to 1 on the robot center only, not around the robot or sensors
		self.visited[self.current[0]][self.current[1]]+=1
		return None
	def offgridcheck(self):
		if not self.checkLeft(self.direction):
			obs = 0
			if self.direction==N:
				if self.current[1]==2:
					if MAX_ROW-1-self.current[0]>8:
						for i in range(MAX_ROW-self.current[0]):
							if self.currentMap[MAX_ROW-1-i][0]==2:
								obs+=1
							else:
								obs = 0
				if obs>6:
					self.current[1] = self.current[1]-1
					for i in range(MAX_ROW-self.current[0]):
						if self.currentMap[MAX_ROW-1-i][0]==2:
							self.currentMap[MAX_ROW-1-i][0]=1
			elif self.direction==E:
				if self.current[0]==2:
					if self.current[1]>5:
						for i in range(MAX_COL):
							if self.currentMap[0][i]==2:
								obs+=1
							else:
								obs = 0
							if obs>6:
								break
						if obs>6:
							self.current[0] = self.current[0]-1
							for j in range(7):
								self.currentMap[0][i-j]=1
			elif self.direction==S:
				if self.current[1]==12:
					if self.current[0]>8:
						for i in range(self.current[0]):
							if self.currentMap[i][14]==2:
								obs+=1
							else:
								obs = 0
				if obs>6:
					self.current[1] = self.current[1]+1
					for i in range(self.current[0]):
						if self.currentMap[i][14]==2:
							self.currentMap[i][14]=1
			elif self.direction==W:
				if self.current[0]==17:
					if self.current[1]<9:
						for i in range(MAX_COL):
							if self.currentMap[19][i]==2:
								obs+=1
							else:
								obs = 0
							if obs>6:
								break
						if obs>6:
							self.current[0] = self.current[0]+1
							for j in range(7):
								self.currentMap[19][i-j]=1
	def get_movement(self, obstacles):
		if self.update:
			self.checkObstacle(obstacles)
			self.offgridcheck()
		logging.info(self.currentMap)
		fin = self.explore_()
		if not fin:
			if not self.rpi:
				return self.send_movement()
			return self.previousMvmt + "V"
		else:
			m, s = divmod(time.time()-self.startTime, 60)
			m = int(m)
			s = int(s)
			exploredtime = '{:02d}:{:02d}'.format(m,s)
			shortestpath = ""
			for i in self.path:
				shortestpath+=str(i)
			return "T|"+shortestpath+'|'+str(exploredtime)+'{:02d}'.format(int(self.explored))
	def send_movement(self):
		dire = ''
		if self.direction==N:
			dire = 'N'
		elif self.direction==E:
			dire = 'E'
		elif self.direction==S:
			dire = 'S'
		else:
			dire = 'W'
		mvstr = ''
		if not self.previousMvmt==FRWD:
			mvstr += 'R'+dire
		else:
			mvstr+='M'+dire+'1'
		return mvstr
	#update unexplored map and current map to explored
	def setExplored(self,explored):
		self.currentMap[explored[0],explored[1]] = 1
	#set obstacles
	def setObstacle(self,obstacle):
		if not (obstacle[0]>=START[0]-1 and obstacle[0]<=START[0]+1 and obstacle[1]>=START[1]-1 and obstacle[1]<=START[1]+1) and not (obstacle[0]>=GOAL[0]-1 and obstacle[0]<=GOAL[0]+1 and obstacle[1]>=GOAL[1]-1 and obstacle[1]<=GOAL[1]+1) and not (obstacle[0]>=self.waypoint[0]-1 and obstacle[0]<=self.waypoint[0]+1 and obstacle[1]>=self.waypoint[1]-1 and obstacle[1]<=self.waypoint[1]+1) and not self.ghost[obstacle[0]][obstacle[1]]>0:
			self.currentMap[obstacle[0],obstacle[1]]=2 
	#translate map grid to mdf string
	def mapToMDF(self):
		mdf = ''
		for i in range(MAX_ROW):
			for j in range(MAX_COL):
				if self.currentMap[MAX_ROW-1-i][j]>0:
					mdf += str(1)
				else:
					mdf += str(0)
		self.mdf1 = '11'+mdf+'11'
		mdf = ''
		for i in range(MAX_ROW):
			for j in range(MAX_COL):
				if self.currentMap[MAX_ROW-1-i][j]>0:
					mdf  += str(self.currentMap[MAX_ROW-1-i][j] - 1)
		while len(mdf)%4!=0:
			mdf += str(0)
		self.mdf2 = mdf
		self.mdf1 = '%0*X' % ((len(self.mdf1) + 3) // 4, int(self.mdf1, 2))
		self.mdf2 = '%0*X' % ((len(self.mdf2) + 3) // 4, int(self.mdf2, 2))
	def sendMDF(self):
		self.mapToMDF()
		mdf = 'MDF|'+self.mdf1+'|'+self.mdf2+'|'
		robotDirection=""
		r = self.current[0]
		c = self.current[1]
		if self.direction==N:
			robotDirection = "N"
		elif self.direction==E:
			robotDirection = "E"
		elif self.direction==S:
			robotDirection = "S"
		elif self.direction==W:
			robotDirection = "W"
		mdf = mdf + robotDirection + '|' + str(r)+'|' +str(c)
		return mdf
	#calculate % of exploration phase
	def calculateMapExplored(self):
		self.explored = int((np.sum(self.currentMap !=0 )/3))
	#get coordinates based on which sensor position and current robot orientation
	def getSensorCoordinates(self,sensor,free):
		sensorX=0
		sensorY=0
		#left bottom sensor
		if sensor==0:
			if free == 0:
				if self.direction==N:
					sensorX = self.current[0]+1
					sensorY = self.current[1]-2
				elif self.direction==E:
					sensorX = self.current[0]-2
					sensorY = self.current[1]-1
				elif self.direction==S:
					sensorX = self.current[0]-1
					sensorY = self.current[1]+2
				elif self.direction==W:
					sensorX = self.current[0]+2
					sensorY = self.current[1]+1
			elif free == 1:
				if self.direction==N:
					sensorX = self.current[0]+1
					sensorY = self.current[1]-3
				elif self.direction==E:
					sensorX = self.current[0]-3
					sensorY = self.current[1]-1
				elif self.direction==S:
					sensorX = self.current[0]-1
					sensorY = self.current[1]+3
				elif self.direction==W:
					sensorX = self.current[0]+3
					sensorY = self.current[1]+1
			else:			
				if self.direction==N:
					sensorX = self.current[0]+1
					sensorY = self.current[1]-4
				elif self.direction==E:
					sensorX = self.current[0]-4
					sensorY = self.current[1]-1
				elif self.direction==S:
					sensorX = self.current[0]-1
					sensorY = self.current[1]+4
				elif self.direction==W:
					sensorX = self.current[0]+4
					sensorY = self.current[1]+1
		#left top sensor
		elif sensor==1:
			if free == 0:
				if self.direction==N:
					sensorX = self.current[0]-1
					sensorY = self.current[1]-2
				elif self.direction==E:
					sensorX = self.current[0]-2
					sensorY = self.current[1]+1
				elif self.direction==S:
					sensorX = self.current[0]+1
					sensorY = self.current[1]+2
				elif self.direction==W:
					sensorX = self.current[0]+2
					sensorY = self.current[1]-1
			elif free == 1:
				if self.direction==N:
					sensorX = self.current[0]-1
					sensorY = self.current[1]-3
				elif self.direction==E:
					sensorX = self.current[0]-3
					sensorY = self.current[1]+1
				elif self.direction==S:
					sensorX = self.current[0]+1
					sensorY = self.current[1]+3
				elif self.direction==W:
					sensorX = self.current[0]+3
					sensorY = self.current[1]-1
			else:
				if self.direction==N:
					sensorX = self.current[0]-1
					sensorY = self.current[1]-3
				elif self.direction==E:
					sensorX = self.current[0]-3
					sensorY = self.current[1]+1
				elif self.direction==S:
					sensorX = self.current[0]+1
					sensorY = self.current[1]+3
				elif self.direction==W:
					sensorX = self.current[0]+3
					sensorY = self.current[1]-1
		#front left sensor
		elif sensor==2:
			if free == 0:
				if self.direction==N:
					sensorX = self.current[0]-2
					sensorY = self.current[1]-1
				elif self.direction==E:
					sensorX = self.current[0]-1
					sensorY = self.current[1]+2
				elif self.direction==S:
					sensorX = self.current[0]+2
					sensorY = self.current[1]+1
				elif self.direction==W:
					sensorX = self.current[0]+1
					sensorY = self.current[1]-2
			elif free == 1:
				if self.direction==N:
					sensorX = self.current[0]-3
					sensorY = self.current[1]-1
				elif self.direction==E:
					sensorX = self.current[0]-1
					sensorY = self.current[1]+3
				elif self.direction==S:
					sensorX = self.current[0]+3
					sensorY = self.current[1]+1
				elif self.direction==W:
					sensorX = self.current[0]+1
					sensorY = self.current[1]-3
			else:
				if self.direction==N:
					sensorX = self.current[0]-4
					sensorY = self.current[1]-1
				elif self.direction==E:
					sensorX = self.current[0]-1
					sensorY = self.current[1]+4
				elif self.direction==S:
					sensorX = self.current[0]+4
					sensorY = self.current[1]+1
				elif self.direction==W:
					sensorX = self.current[0]+1
					sensorY = self.current[1]-4
		#front middle sensor
		elif sensor==3:
			if free == 0:
				if self.direction==N:
					sensorX = self.current[0]-2
					sensorY = self.current[1]
				elif self.direction==E:
					sensorX = self.current[0]
					sensorY = self.current[1]+2
				elif self.direction==S:
					sensorX = self.current[0]+2
					sensorY = self.current[1]
				elif self.direction==W:
					sensorX = self.current[0]
					sensorY = self.current[1]-2
			elif free == 1:
				if self.direction==N:
					sensorX = self.current[0]-3
					sensorY = self.current[1]
				elif self.direction==E:
					sensorX = self.current[0]
					sensorY = self.current[1]+3
				elif self.direction==S:
					sensorX = self.current[0]+3
					sensorY = self.current[1]
				elif self.direction==W:
					sensorX = self.current[0]
					sensorY = self.current[1]-3
			else:
				if self.direction==N:
					sensorX = self.current[0]-4
					sensorY = self.current[1]
				elif self.direction==E:
					sensorX = self.current[0]
					sensorY = self.current[1]+4
				elif self.direction==S:
					sensorX = self.current[0]+4
					sensorY = self.current[1]
				elif self.direction==W:
					sensorX = self.current[0]
					sensorY = self.current[1]-4
		#front right sensor
		elif sensor==4:
			if free == 0:
				if self.direction==N:
					sensorX = self.current[0]-2
					sensorY = self.current[1]+1
				elif self.direction==E:
					sensorX = self.current[0]+1
					sensorY = self.current[1]+2
				elif self.direction==S:
					sensorX = self.current[0]+2
					sensorY = self.current[1]-1
				elif self.direction==W:
					sensorX = self.current[0]-1
					sensorY = self.current[1]-2
			elif free == 1:
				if self.direction==N:
					sensorX = self.current[0]-3
					sensorY = self.current[1]+1
				elif self.direction==E:
					sensorX = self.current[0]+1
					sensorY = self.current[1]+3
				elif self.direction==S:
					sensorX = self.current[0]+3
					sensorY = self.current[1]-1
				elif self.direction==W:
					sensorX = self.current[0]-1
					sensorY = self.current[1]-3
			else:
				if self.direction==N:
					sensorX = self.current[0]-4
					sensorY = self.current[1]+1
				elif self.direction==E:
					sensorX = self.current[0]+1
					sensorY = self.current[1]+4
				elif self.direction==S:
					sensorX = self.current[0]+4
					sensorY = self.current[1]-1
				elif self.direction==W:
					sensorX = self.current[0]-1
					sensorY = self.current[1]-4
		#right middle sensor
		elif sensor==5:
			if free == 0:
				if self.direction==N:
					sensorX = self.current[0]
					sensorY = self.current[1]+2
				elif self.direction==E:
					sensorX = self.current[0]+2
					sensorY = self.current[1]
				elif self.direction==S:
					sensorX = self.current[0]
					sensorY = self.current[1]-2
				elif self.direction==W:
					sensorX = self.current[0]-2
					sensorY = self.current[1]
			elif free == 1:
				if self.direction==N:
					sensorX = self.current[0]
					sensorY = self.current[1]+3
				elif self.direction==E:
					sensorX = self.current[0]+3
					sensorY = self.current[1]
				elif self.direction==S:
					sensorX = self.current[0]
					sensorY = self.current[1]-3
				elif self.direction==W:
					sensorX = self.current[0]-3
					sensorY = self.current[1]
			else:
				if self.direction==N:
					sensorX = self.current[0]
					sensorY = self.current[1]+4
				elif self.direction==E:
					sensorX = self.current[0]+4
					sensorY = self.current[1]
				elif self.direction==S:
					sensorX = self.current[0]
					sensorY = self.current[1]-4
				elif self.direction==W:
					sensorX = self.current[0]-4
					sensorY = self.current[1]
		return [sensorX,sensorY]
	#check obstacles using given method from simulator/robot and update map if any
	#0,1,2,3,9
	#0 for obstacle right in front
	#1 for obstacle 1 grid away
	#
	def checkObstacle(self, obstacles):
		sensors = ''
		sensors = obstacles
		if not self.rpi:
			simMap = self.createSimMap()
			obStr = ''
			for i in range(6):
				sensor = False
				for j in range(3):
					coords1 = self.getSensorCoordinates(i,j)
					if coords1[0]>=0 and coords1[0] <MAX_ROW and coords1[1]>=0 and coords1[1]<MAX_COL:
						if simMap[coords1[0],coords1[1]]==1:
							obStr+=str(j)
							sensor = True
							break
					elif coords1[0]==-1 or coords1[0]==MAX_ROW and coords1[1]>=1 and coords1[1]<MAX_COL:
						obStr+=str(j)
						sensor = True
						break
					elif coords1[1]==-1 or coords1[1]==MAX_COL and coords1[0]>=0 and coords1[0]<MAX_ROW :
						obStr+=str(j)
						sensor = True
						break
				if not sensor:
					obStr+='9'
			sensors = obStr
		if self.phase==1:
			i = 0
			for obs in sensors:
				if obs=='9':
					#Tweak left and right sensors to only read in 2 grid
					# or i==1 or i==5
					if i==0:
						for j in range(2):
							coords = self.getSensorCoordinates(i,j)
							if self.valExplored([coords]):
								self.setExplored(coords)
					else:
						for j in range(3):
							coords = self.getSensorCoordinates(i,j)
							if self.valExplored([coords]):
								self.setExplored(coords)
				elif obs=='1':
					coords = self.getSensorCoordinates(i,int(obs))
					if self.valObstacles([coords]):
						self.setObstacle(coords)
					coords = self.getSensorCoordinates(i,0)
					if self.valExplored([coords]):
						self.setExplored(coords)
				elif obs=='0':
					coords = self.getSensorCoordinates(i,int(obs))
					if self.valObstacles([coords]):
						self.setObstacle(coords)
				elif obs=='2':
					coords = self.getSensorCoordinates(i,int(obs))
					if self.valObstacles([coords]):
						self.setObstacle(coords)
					coords = self.getSensorCoordinates(i,0)
					if self.valExplored([coords]):
						self.setExplored(coords)
					coords = self.getSensorCoordinates(i,1)
					if self.valExplored([coords]):
						self.setExplored(coords)
				i+=1
		elif self.phase==2:
			i = 2
			for j in range(3):
				if sensors[i]=='9':
					for j in range(3):
						coords = self.getSensorCoordinates(i,j)
						if self.valExplored([coords]):
							self.setExplored(coords)
				elif sensors[i]=='1':
					coords = self.getSensorCoordinates(i,int(sensors[i]))
					if self.valObstacles([coords]):
						self.setObstacle(coords)
					coords = self.getSensorCoordinates(i,0)
					if self.valExplored([coords]):
						self.setExplored(coords)
				elif sensors[i]=='0':
					coords = self.getSensorCoordinates(i,int(sensors[i]))
					if self.valObstacles([coords]):
						self.setObstacle(coords)
				elif sensors[i]=='2':
					coords = self.getSensorCoordinates(i,int(sensors[i]))
					if self.valObstacles([coords]):
						self.setObstacle(coords)
					coords = self.getSensorCoordinates(i,0)
					if self.valExplored([coords]):
						self.setExplored(coords)
					coords = self.getSensorCoordinates(i,1)
					if self.valExplored([coords]):
						self.setExplored(coords)
				i+=1
	def createSimMap(self):
		simMap = np.zeros([MAX_ROW,MAX_COL],dtype=int)
		exploredMap = ''
		with open('map/sample.txt') as f:
			for grid in f:
				exploredMap +=grid.strip()
			for i in range(20):
				for j in range(15):
					if exploredMap:
						simMap[i][j]=exploredMap[0]
						exploredMap = exploredMap[1:]
		return simMap
	def valExplored(self,coords):
		for (r,c) in coords:
			if r< 0 or r>=MAX_ROW or c<0 or c>=MAX_COL:
				return False
		return True
	def valObstacles(self,coords):
		for (r, c) in coords:
				# if r == 0 or r == MAX_ROW-1 or c == 0 or c == MAX_COL-1:
				# 	return False
				if r < 0 or r >=MAX_ROW or c < 0 or c >=MAX_COL or self.currentMap[r,c]==2:
					return False
		return True
	#check in the orientation assuming robot is facing north
	def checkNorth(self):
		r, c = self.current
		return [[r - 2, c], [r - 2, c - 1], [r - 2, c + 1]]
	def checkEast(self):
		r, c = self.current
		return [[r, c + 2], [r - 1, c + 2], [r + 1, c + 2]]
	def checkSouth(self):
		r, c = self.current
		return [[r + 2, c], [r + 2, c - 1], [r + 2, c + 1]]
	def checkWest(self):
		r, c = self.current
		return [[r, c - 2], [r - 1, c - 2], [r + 1, c - 2]]
	#check whether movement can be executed, or not 
	#determine virtual wall and robot position
	def valMvmt(self, robotPos):
		for (r, c) in robotPos:
				if r < 0 or r >=MAX_ROW or c < 0 or c >=MAX_COL or self.currentMap[r,c]==2:
					return False
		return True
	#check whether direction is free by checking 20cm in direction from centre of robot
	#then validate whether movement can be done by checking virtual wall
	#check 3 grids north of robot
	def checkFront(self, direction):
		return self.valMvmt(self.getFront(direction))
	#check 3 grids east of robot
	def checkRight(self, direction):
		return self.valMvmt(self.getRight(direction))
	#check 3 grids south of robot
	def checkBottom(self, direction):
		return self.valMvmt(self.getBottom(direction))
	#check 3 grids west of robot
	def checkLeft(self, direction):
		return self.valMvmt(self.getLeft(direction))
	#get coordinates based on robots current orientation
	def getFront(self,direction):
		nextRobotPos = [[]]
		if direction==N:
			nextRobotPos = self.checkNorth()
		elif direction==E:
			nextRobotPos = self.checkEast()
		elif direction==S:
			nextRobotPos = self.checkSouth()
		else:
			nextRobotPos = self.checkWest()
		return nextRobotPos
	def getRight(self,direction):
		nextRobotPos = [[]]
		if direction==N:
			nextRobotPos = self.checkEast()
		elif direction==E:
			nextRobotPos = self.checkSouth()
		elif direction==S:
			nextRobotPos = self.checkWest()
		else:
			nextRobotPos = self.checkNorth()
		return nextRobotPos
	def getLeft(self,direction):
		nextRobotPos = [[]]
		if direction==N:
			nextRobotPos = self.checkWest()
		elif direction==E:
			nextRobotPos = self.checkNorth()
		elif direction==S:
			nextRobotPos = self.checkEast()
		else:
			nextRobotPos = self.checkSouth()
		return nextRobotPos
	def getBottom(self,direction):
		nextRobotPos = [[]]
		if direction==N:
			nextRobotPos = self.checkSouth()
		elif direction==E:
			nextRobotPos = self.checkWest()
		elif direction==S:
			nextRobotPos = self.checkNorth()
		else:
			nextRobotPos = self.checkEast()
		return nextRobotPos
	def checkGoalExplored(self):
		neighbors = self.getNeighbors(GOAL)
		for n in neighbors:
			if self.visited[n[0]][n[1]]>0:
				return True
		return False
	def checkExploredFront(self, indices,direction):
		fl = False
		if direction==N:
			indices[0][0]+=1
		elif direction==E:
			indices[0][1]-=1
		elif direction==S:
			indices[0][0]-=1
		else:
			indices[0][1]+=1
		if self.visited[indices[0][0],indices[0][1]]==1:
			fl = True        
		return fl
	def checkExploredRight(self,indices,direction):
		fl = False
		# for i in range(3):
		if direction==N:
			indices[0][1]-=1
		elif direction==E:
			indices[0][0]-=1
		elif direction==S:
			indices[0][1]+=1
		else:
			indices[0][0]+=1
		if self.visited[indices[0][0],indices[0][1]]==1:
			fl = True
		return fl
	def clearObstacles(self, coords):
		neighbors = self.getNeighbors(coords)
		for n in neighbors:
			if self.currentMap[n[0],n[1]]==2:
				self.currentMap[n[0],n[1]]=1
	def checkFrontObstacles(self, direction):
		obstacles = self.getFront(direction)
		for (r, c) in obstacles:
				if (r+1 == 0 or r-1==MAX_ROW and c+1>=0 and c-1<=MAX_COL) or (c+1 == 0 or c-1 == MAX_COL and r+1>=0 and r-1<=MAX_ROW):
					return True
				elif r>=0 and r<MAX_ROW and c>=0 and c<MAX_COL and self.currentMap[r][c]!=2:
					return False
		return True
	def checkLeftObstacles(self, direction):
		obstacles = self.getLeft(direction)
		for (r, c) in obstacles:
				if (r+1 == 0 or r-1==MAX_ROW and c+1>=0 and c-1<=MAX_COL) or (c+1 == 0 or c-1 == MAX_COL and r+1>=0 and r-1<=MAX_ROW):
					return True
				elif r>=0 and r<MAX_ROW and c>=0 and c<MAX_COL and self.currentMap[r][c]!=2:
					return False
		return True
	def nextMove(self):
		return self.explorationAlgorithm()
	#make sure that robot is following left wall
	#always check in front of robot
	#no infinite loops
	def explorationAlgorithm(self):
		dire = self.direction
		move = ''
		#new algorithm
		#corner calibration as priority
		#move right after calibration
		if self.checkLeftObstacles(dire) and self.checkFrontObstacles(dire) and self.stepCounter>1:
			self.stepCounter = 0
			move = "U"
			return move
		#calibrate before turning right
		elif self.checkLeftObstacles(dire) and not self.checkFront(dire) and self.stepCounter>=2:
			self.stepCounter = 0
			move = "C"
			return move
		#calibrate every 7 steps
		elif self.checkLeftObstacles(dire) and self.stepCounter>=7:
			self.stepCounter = 0
			move = "C"
			return move
		#left hand rule algorithm
		#if can go left, go left if can go straight go straight else turn right
		if self.checkLeft(dire) and self.previousMvmt==FRWD:
			move = LEFT
		elif self.checkFront(dire):
			move = FRWD
		elif self.checkRight(dire):
			move = RIGHT
		else:
			move = TURNA
		#prevent endless loop, try to break by moving until it reaches a wall/osbtacle and then continue left wall hug
		if self.backtrack:
			if self.checkFront(dire):
				move = FRWD
			elif self.checkRight(dire):
				move = RIGHT
				self.backtrack=False
			elif self.checkLeft(dire) and self.previousMvmt==FRWD:
				move = LEFT
		return move
	#move robot coordinates in robot.current
	def mv(self,move):
		self.previousMvmt=move
		if move==FRWD:
			if self.direction==N:
				self.current=np.asarray([self.current[0]-1,self.current[1]])
			elif self.direction==E:
				self.current=np.asarray([self.current[0],self.current[1]+1])
			elif self.direction==S:
				self.current=np.asarray([self.current[0]+1,self.current[1]])
			else:
				self.current=np.asarray([self.current[0],self.current[1]-1])
			if not self.explored==100 and time.time() < self.endTime:
				self.updateRobotPosition()
		elif move==RIGHT or move=="U":
			if self.direction==N:
				self.direction=E
			elif self.direction==E:
				self.direction=S
			elif self.direction==S:
				self.direction=W
			else:
				self.direction=N
		elif move==LEFT:
			if self.direction==N:
				self.direction=W
			elif self.direction==E:
				self.direction=N
			elif self.direction==S:
				self.direction=E
			else:
				self.direction=S			
		elif move==TURNA:
			if self.direction==N:
				self.direction=S
			elif self.direction==E:
				self.direction=W
			elif self.direction==S:
				self.direction=N
			else:
				self.direction=E
	def move(self):
		move = self.nextMove()
		self.mv(move)
		self.calculateMapExplored()
		time.sleep(self.stepTime)
		return move
	def explore(self,userLim,exploreLim,stepTime):
		#set user time limit
		self.userLim = userLim
		#set exploration percentage
		self.exploreLim = exploreLim
		#set step delay
		self.stepTime = stepTime
	def setWaypt(self,waypoint):
		self.waypoint = (int(waypoint[0]),int(waypoint[1]))
	def setGoal(self,goal):
		GOAL = [goal[0],goal[1]]
	@run_once
	def start(self):
		self.startTime = time.time()
		self.endTime = self.startTime+self.timeLim*60
		self.userTime = self.startTime+self.userLim
		self.stepCounter = 0
		self.steps = 0
	def explore_(self):
		self.start()
		if self.visited[18][1]>1 and self.phase==1:
			self.phase = 2
		if self.explored==100 and (self.phase==1 or self.phase==2):
			self.phase = 3
		if time.time() >= self.endTime:
			return True
		if self.explored==100 and self.checkGoalExplored() and self.phase==4 or time.time() >= self.userTime and self.checkGoalExplored():
			if self.explored==100:
				self.update = False
			if self.current[0] == START[0] and self.current[1] == START[1]:
				if self.calib==0:
					if self.direction==E:
						self.mv(LEFT)
					elif self.direction==S:
						self.mv(TURNA)
					elif self.direction==W:
						self.mv(RIGHT)
					elif self.direction==N:
						self.calib=1
						self.mv(LEFT)
						self.previousMvmt = LEFT+"C"
				elif self.calib==1:
					self.mv("U")
					self.previousMvmt = "U"
					self.calib=2
				elif self.calib==2:
						exploredMap = self.currentMap.copy()
						for r in range(MAX_ROW):
							for c in range(MAX_COL):
								if exploredMap[r][c]>0:
									exploredMap[r][c] = exploredMap[r][c]-1
						start = (START[0],START[1])
						goal = (GOAL[0],GOAL[1])
						sp = ShortestPathRB(exploredMap,start,goal,self.direction)
						startDirection = sp.calibrationStartDirection(start,self.waypoint,goal)
						self.path = sp.ActionSequence(start,self.waypoint,startDirection)
						wayptToGoal = sp.ActionSequence(self.waypoint, goal, sp.Dir)
						logging.info(self.path)
						logging.info(wayptToGoal)
						try:
							if self.path[len(self.path)-2][0]=='S' and wayptToGoal[0][0]=='S':
								if (int(self.path[len(self.path)-2][1])+int(wayptToGoal[0][1]))<10:
									m = int(self.path[len(self.path)-2][1])+int(wayptToGoal[0][1])
									self.path[len(self.path)-2] = "S"+str(m)
									wayptToGoal = wayptToGoal[1:]
						except:
							logging.info("Waypttogoal is empty")
						mvmtstr = ""
						for mvmt in self.path:
							mvmtstr += str(mvmt)
						for mvmt in wayptToGoal:
							mvmtstr += str(mvmt)
						self.path = ""
						self.path = mvmtstr
						if startDirection == E:
							self.mv(RIGHT)
							self.calib = 3
						else:
							self.mapToMDF()
							logging.info("MDF|"+self.mdf1+"|"+self.mdf2)
							return True
						return
				elif self.calib==3:
					self.mapToMDF()
					logging.info("MDF|"+self.mdf1+"|"+self.mdf2)
					return True
				time.sleep(self.stepTime)
				return
			else:
				self.finished()
				if self.rpi:
					mvmtstr = ""
					for mvmt in self.path:
						mvmtstr += str(mvmt)
					self.previousMvmt=mvmtstr
					return
				else:
					try:
						if self.path[0][0]!='S':
							move = self.path[0]
							self.path.pop(0)
							self.mv(move)
						else:
							if len(self.path[0])>2:
								steps = int(self.path[0][2])
								steps-=1
								if steps==0:
									self.path[0]='S'+str(9)	
								else:
									self.path[0][2] = str(steps)
								self.mv(FRWD)
							else:
								steps = int(self.path[0][1])
								steps-=1						
								self.path[0] = 'S'+str(steps)
								if steps==0:
									self.path.pop(0)
								self.mv(FRWD)
						time.sleep(self.stepTime)
						return
					except:
						return
		else:
			if self.phase==1:
				if self.steps>30:
					self.steps=0
					for r in range(MAX_ROW):
						for c in range(MAX_COL):
							if self.visited[r][c]>0:
								self.visited[r][c] = 1
				if self.visited[self.current[0]][self.current[1]]>3:
					self.backtrack=True
				mv = self.move()
				if not mv =="C" and not mv=="U":
					self.stepCounter += 1
					self.steps+=1
			elif self.phase==2:
				self.getUnexplored()
				if self.goto:
					if self.checkLeftObstacles(self.direction) and self.checkFrontObstacles(self.direction) and self.stepCounter>1:
						self.stepCounter = 0
						move = "U"
						self.mv(move)
						if self.path[0][0]=='R':
							self.path.pop(0)
						return
					elif self.checkLeftObstacles(self.direction) and not self.checkFront(self.direction) and self.stepCounter>=2:
						self.stepCounter = 0
						move = "C"
						self.mv(move)
						return
					elif self.checkLeftObstacles(self.direction) and self.stepCounter>=7:
						self.stepCounter = 0
						move = "C"
						self.mv(move)
						return
					self.getPath()
					if self.path:
						if self.path[0][0]!='S':
							move = self.path.pop(0)
							self.mv(move)
						else:
							steps = int(self.path[0][1])
							steps-=1						
							self.path[0] = 'S'+str(steps)
							if steps==0:
								self.path.pop(0)
							self.mv(FRWD)
					else:
						self.explore_()
						return
					time.sleep(self.stepTime)
					self.stepCounter+=1
				else:
					if self.checkGoalExplored():
						self.calculateMapExplored()
						self.explore_()
					else:
						self.currentMap[GOAL[0]][GOAL[1]]=0
						self.explore_()
				return
			elif self.phase==3:
				self.getObstacles()
				if self.goto or self.path:
					if self.checkLeftObstacles(self.direction) and self.checkFrontObstacles(self.direction) and self.stepCounter>1:
						self.stepCounter = 0
						move = "U"
						self.mv(move)
						if self.path[0][0]=='R':
							self.path.pop(0)
						return
					elif self.checkLeftObstacles(self.direction) and not self.checkFront(self.direction) and self.stepCounter>=2:
						self.stepCounter = 0
						move = "C"
						self.mv(move)
						return
					elif self.checkLeftObstacles(self.direction) and self.stepCounter>=7:
						self.stepCounter = 0
						move = "C"
						self.mv(move)
						return
					if self.path:
						if self.path[0][0]!='S':
							move = self.path.pop(0)
							self.mv(move)
						else:
							steps = int(self.path[0][1])
							steps-=1						
							self.path[0] = 'S'+str(steps)
							if steps==0:
								self.path.pop(0)
							self.mv(FRWD)
					else:
						self.getArrowPath()	
						self.goto.pop(0)
						self.explore_()
					time.sleep(self.stepTime)
					self.stepCounter+=1
				else:
					self.phase = 4
					self.explore_()
				return
	def valPath(self, robotPos):
		for (r, c) in robotPos:
				if r >0 and r <MAX_ROW-1 and c > 0 and c <MAX_COL-1 and not self.currentMap[r][c]==2:
					return True
		return False
	def getPath(self):
		exploredPos = []
		r = self.goto[0][0]
		c = self.goto[0][1]
		j = 2
		try:
			for i in range(3):
				if r >=0 and r<=2 and c>=12 and c<=14:
					exploredPos=(r,c)
					break
				north = (r-j,c)
				northleft = (r-j,c-1)
				northright = (r-j,c+1)
				south = (r+j,c)
				southleft = (r+j,c-1)
				southright = (r+j,c+1)
				east = (r,c+j)
				easttop = (r-1,c+j)
				eastbtm = (r+1,c+j)
				west = (r,c-j)
				westtop = (r-1,c-j)
				westbtm = (r+1,c-j)
				j+=1
				neighbors = self.getNeighbors(north)
				isObs = False
				for n in neighbors:
					if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
						isObs = True
				if not isObs and self.valPath([north]):
					exploredPos = north
				else:
					neighbors = self.getNeighbors(south)
					isObs = False
					for n in neighbors:
						if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
							isObs = True
					if not isObs and self.valPath([south]):
						exploredPos = south
					else:
						neighbors = self.getNeighbors(east)
						isObs = False
						for n in neighbors:
							if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
								isObs = True
						if not isObs and self.valPath([east]):
							exploredPos = east
						else:
							neighbors = self.getNeighbors(west)
						isObs = False
						for n in neighbors:
							if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
								isObs = True
						if not isObs and self.valPath([west]):
							exploredPos = west
						else:
							neighbors = self.getNeighbors(northleft)
							isObs = False
							for n in neighbors:
								if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
									isObs = True
							if not isObs and self.valPath([northleft]):
								exploredPos = northleft
							else:
								neighbors = self.getNeighbors(northright)
								isObs = False
								for n in neighbors:
									if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
										isObs = True
								if not isObs and self.valPath([northright]):
									exploredPos = northright
								else:
									neighbors = self.getNeighbors(southleft)
									isObs = False
									for n in neighbors:
										if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
											isObs = True
									if not isObs and self.valPath([southleft]):
										exploredPos = southleft
									else:
										neighbors = self.getNeighbors(southright)
										isObs = False
										for n in neighbors:
											if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
												isObs = True
										if not isObs and self.valPath([southright]):
											exploredPos = southright
										else:
											neighbors = self.getNeighbors(easttop)
											isObs = False
											for n in neighbors:
												if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
													isObs = True
											if not isObs and self.valPath([easttop]):
												exploredPos = easttop
											else:
												neighbors = self.getNeighbors(eastbtm)
												isObs = False
												for n in neighbors:
													if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
														isObs = True
												if not isObs and self.valPath([eastbtm]):
													exploredPos = eastbtm
												else:
													neighbors = self.getNeighbors(westtop)
													isObs = False
													for n in neighbors:
														if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
															isObs = True
													if not isObs and self.valPath([westtop]):
														exploredPos = westtop
													else:
														neighbors = self.getNeighbors(westbtm)
														isObs = False
														for n in neighbors:
															if self.valExplored([n]) and self.currentMap[n[0]][n[1]]==2:
																isObs = True
														if not isObs and self.valPath([westbtm]):
															exploredPos = westbtm
				if not exploredPos==[]:
					break
		except:
			logging.info("FAILED")
			self.currentMap[self.goto[0][0]][self.goto[0][1]]=2
			return
		try:
			if exploredPos[0]==self.current[0] and exploredPos[1]==self.current[1]:
				self.currentMap[self.goto[0][0]][self.goto[0][1]]=2
				return
		except:
			logging.info("No explored position to go to")
			self.currentMap[self.goto[0][0]][self.goto[0][1]]=2
			return
		exploredMap = self.currentMap.copy()
		for r in range(MAX_ROW):
			for c in range(MAX_COL):
				if exploredMap[r][c]>0:
					exploredMap[r][c] = exploredMap[r][c]-1
		try:
			sp = ShortestPathRB(exploredMap,(self.current[0],self.current[1]),(exploredPos),self.direction)
			self.path = sp.ActionSequence((self.current[0],self.current[1]),(exploredPos), self.direction)
			self.lookFront(self.goto[0],exploredPos,sp.Dir)	
		except:
			logging.info("KeyError")
			self.currentMap[self.goto[0][0]][self.goto[0][1]]=1
	def lookFront(self,goto,exploredPos,direction): 
		if goto[0]+1==exploredPos[0] or goto[0]-1==exploredPos[0] or goto[0]==exploredPos[0]:
			if goto[1]<exploredPos[1]:
				if not direction==W:
					if direction==E:
						self.path.append(TURNA)
					elif direction==N:
						self.path.append(LEFT)
					else:
						self.path.append(RIGHT)
			else:
				if not direction==E:
					if direction==W:
						self.path.append(TURNA)
					elif direction==N:
						self.path.append(RIGHT)
					else:
						self.path.append(LEFT)
		elif goto[0]<exploredPos[0]:
			if not direction==N:
				if direction==S:
					self.path.append(TURNA)
				elif direction==E:
					self.path.append(LEFT)
				else:
					self.path.append(RIGHT)
		else:
			if not direction==S:
				if direction==N:
					self.path.append(TURNA)
				elif direction==E:
					self.path.append(RIGHT)
				else:
					self.path.append(LEFT)
	def getUnexplored(self):
		self.goto = []
		for r in range(MAX_ROW):
			for c in range(MAX_COL):
				if self.currentMap[19-r][c]==0:
					self.goto.append((19-r,c))
	def lookLeft(self,arrowPos,direction,north,south,east,west):
		if arrowPos==north:	
			if direction==N:
				self.path.append(LEFT)
			elif direction==E:
				self.path.append(TURNA)
			elif direction==S:
				self.path.append(RIGHT)
		elif arrowPos==south:
			if direction==N:
				self.path.append(RIGHT)
			elif direction==W:
				self.path.append(TURNA)
			elif direction==S:
				self.path.append(LEFT)
		elif arrowPos==east:
			if direction==W:
				self.path.append(RIGHT)
			elif direction==E:
				self.path.append(LEFT)
			elif direction==S:
				self.path.append(TURNA)
		elif arrowPos==west:
			if direction==N:
				self.path.append(TURNA)
			elif direction==E:
				self.path.append(RIGHT)
			elif direction==W:
				self.path.append(LEFT)
	def getArrowPath(self):
		arrowPos = []
		r = self.goto[0][0]
		c = self.goto[0][1]
		north = (r-2,c)
		south = (r+2,c)
		east = (r,c+2)
		west = (r,c-2)
		neighbors = self.getNeighbors(north)
		isObs = False
		for n in neighbors:
			if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
				isObs = True
		if not isObs and self.valPath([north]) and self.visited[north[0]][north[1]]==0:
			arrowPos.append(north)
		neighbors = self.getNeighbors(south)
		isObs = False
		for n in neighbors:
			if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
				isObs = True
		if not isObs and self.valPath([south]) and self.visited[south[0]][south[1]]==0:
			arrowPos.append(south)
		neighbors = self.getNeighbors(east)
		isObs = False
		for n in neighbors:
			if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
				isObs = True
		if not isObs and self.valPath([east]) and self.visited[east[0]][east[1]]==0:
			arrowPos.append(east)
		neighbors = self.getNeighbors(west)
		isObs = False
		for n in neighbors:
			if self.valExplored([n]) and not self.currentMap[n[0]][n[1]]==1:
				isObs = True
		if not isObs and self.valPath([west]) and self.visited[west[0]][west[1]]==0:
			arrowPos.append(west)
		exploredMap = self.currentMap.copy()
		for r in range(MAX_ROW):
			for c in range(MAX_COL):
				if exploredMap[r][c]>0:
					exploredMap[r][c] = exploredMap[r][c]-1
		try:
			sp = ShortestPathRB(exploredMap,(self.current[0],self.current[1]),(arrowPos[0]),self.direction)
			self.path = sp.ActionSequence((self.current[0],self.current[1]),(arrowPos[0]),self.direction)
			self.lookLeft(arrowPos[0],sp.Dir,north,south,east,west)
			if len(arrowPos)>1:
				j = 1
				for i in range(len(arrowPos-1)):
					self.path.append(sp.ActionSequence((arrowPos[j-1]),(arrowPos[j]),self.direction))
					self.lookLeft(arrowPos[j],sp.Dir,north,south,east,west)
		except:
			logging.info("KeyError/ No Arrow Position")
	@run_once
	def getObstacles(self):
		self.goto = []
		for r in range(MAX_ROW):
			for c in range(MAX_COL):
				if self.currentMap[19-r][c]==2:
					self.goto.append((19-r,c))
	@run_once
	def finished(self):
		self.clearObstacles(self.waypoint)
		self.clearObstacles(GOAL)
		self.clearObstacles(START)
		exploredMap = self.currentMap.copy()
		for r in range(MAX_ROW):
			for c in range(MAX_COL):
				if exploredMap[r][c]>0:
					exploredMap[r][c] = exploredMap[r][c]-1
		sp = ShortestPathRB(exploredMap,(self.current[0],self.current[1]),(START[0],START[1]),self.direction)
		self.path = sp.ActionSequence((self.current[0],self.current[1]),(START[0],START[1]), self.direction)
		if self.rpi:
			self.current = [18,1]
			self.direction=sp.Dir
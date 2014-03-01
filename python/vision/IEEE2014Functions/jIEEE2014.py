import cv2
import numpy as np
##TODO:
#Optimize --
# - Port mathy, timesuck functions (perp, segIntersect) to C++, use boost.python
# - If many lines, skip waste ops like typecheck
# - For All intersect check is suboptimal
# - Float conversion _WILL_ affect performance
# - Consider threshold intersection instead of infinite-extension intersection
# - - If intX < min(x1,x2) - 10 OR  intX > max(x1,x1) + 10
# - Ensure that START is _always_ the left,bottom most pt

#Improve -- 
# - Under real circumstances, there could be multiple nearby lines that intersect the rectangles
# - Respond by taking blue-edge lines
# - Use Built-in 'Reduce()'
def perp(a):
	b = np.empty_like(a)
	b[0] = -a[1]
	b[1] = a[0]
	return b
	
class polyGroup():
	#Connections do not need to be directed, but they do need to be (ha-ha) connected
	#Should be formatted as a looped-list (Like a linked-list, but in a loop)
	##Duplicated edges should be removed
	##If edge[n] intersects any edge[k] where |k-n| > 1, the group is closed with order len(edgeList) - k
	
	def __init__(self,anchorVertex, endVertex):
		#self.anchor = anchorVertex
		self.closed = False
		self.graph = {anchorVertex:endVertex}
		self.anchor = self.graph.itervalues().next()
				
		
		print initEdge.length
	#Each edge that's added should be sequential
	
	def addEdge(self,edge):
		#Edge[0] and Edge[1] are points for a directed graph
		
		##This needs work
		#TODO
		if((edge[0] in self.graph)):
		
			#Do lone examination when back from dinner
		
		
		
		
			if (edge[1] in self.graph) and (self.graph[edge[0]] != edge[1]):
				self.closed = True
			if(edge[1] in self.graph.values()):
				self.graph[edge[1]] = edge[0]
			
		elif((edge[1] in self.edges)):
		
			if (edge[0] in self.graph) and (self.graph[edge[1]] != edge[0]):
				self.closed = True
			if(edge[0] in self.graph.values()):
				self.graph[edge[0]] = edge[1]
			

		
		#self.edges.append(edge)
		
		return(self.edges)

	def appendGroup(self,group):
	
		#for k in   
	
		self.edges.extend(group.edges)
		return(self.edges)
		
		
	
	
class lineObj():
		
	def __init__(self, inLine):
		
		inLine = map(float,inLine) #Convert all elements to float
		self.start = (inLine[0],inLine[1])
		self.end = (inLine[2],inLine[3])
		sub = inLine[2] - inLine[0]
		self.slope = None
		
		if sub:
			self.slope = float (inLine[3] - inLine[1])/(inLine[2] - inLine[0])#Slope is the same regardless of the order of subtraction
		
		vector = np.subtract( self.start, self.end )
		self.length = np.linalg.norm( vector )	
		
		#If there are more bad intersections than good intersections, this will still not be more efficient in the segIntersect fcn
		self.minSelfx =  min(self.start[0],self.end[0])
		self.maxSelfx = max(self.start[0],self.end[0])
	
		self.minSelfy = min(self.start[1],self.end[1])
		self.maxSelfy = max(self.start[1],self.end[1])

		#TODO: Use GROUP class
		self.group = set()
		self.group.add(self)
		self.intersections = []
	def groupAdd(self,otherLine):
		if(len(otherLine.group) == 1):
			self.group.add(otherLine)
		else:
			self.group.union(otherLine.group)
		
		
		return self.group
		
 	def segIntersect(self,otherLine, maxDist):
		#If infinitely extended self intersects with infinitely extended otherLine, return pt
		#Else: return False
		
		if type(otherLine) != type(self):
			print "This function requires a line object"
			return (None)
		if (self.length == 0) or (otherLine.length == 0):
			return (None)
		
		da = np.subtract(self.end,self.start)
		db = np.subtract(otherLine.end,otherLine.start)
		dp = np.subtract(self.start,otherLine.start)
		dap = perp(da)
		denom = np.dot(dap,db)
		num = np.dot(dap,dp)
		
		
		if(denom != 0):
			pt = tuple(map(int,(num / denom)*db + otherLine.start))
			#The following lines are unforgiveably long. I apologize. Could save ops by breaking up
			if((pt[0]  < self.minSelfx - maxDist) or (pt[0] > self.maxSelfx + maxDist) or (pt[1] < self.minSelfy - maxDist) or (pt[1] > self.maxSelfy + maxDist)):
				return (None)
			elif((pt[0]  < otherLine.minSelfx - maxDist) or (pt[0] > otherLine.maxSelfx + maxDist) or (pt[1] < otherLine.minSelfy - maxDist) or (pt[1] > otherLine.maxSelfy + maxDist)):
				return (None)
			else:
				self.intersections.append(pt)
				return pt
		else:
			return (None)

	def getStart(self):
		return self.start
	def getEnd(self):
		return self.end
	def getSlope(self):
		return self.slope
		
		
#Test:
if __name__ == "__main__":
	dbgLine = 0
	dbgGrp = 1

	print "Debugging Line Class"
	Line1 = lineObj([0,0,5,5])
	Line2 = lineObj([5,0,0,5])
	print Line1.segIntersect(Line2,5)

	Line3 = lineObj([0,0,8,4])
	Line4 = lineObj([4,6,5,1])
	test3 = Line3.segIntersect(Line4,4)
	test4 = Line4.segIntersect(Line3,5)
	print test3,test4


	Line5 = lineObj([0,0,2,2])
	print Line5.segIntersect(Line2,5)
	Line6 = lineObj([100,0,0,100])
	print Line6.segIntersect(Line5,10)
	print "Debugging Group Class"
	grp1 = polyGroup(Line1)


	
	
	

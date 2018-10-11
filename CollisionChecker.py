import math
import bisection
def findSidesAtStart (centerLocation, sideLength):
    """
    finds the corners of a square that describes the vehicle
    :param centerLocation: is a tuple of the form (dbl,dbl) where the first element is the x location of the center and
    the second element is the y location of the center
    :param sideLength: is a dbl describing the length of each side 
    :return: returns 2 tuples of the form (dbl,dbl) where the first element is the max and the second element is the min
     of the sides of the square, the first tuples describes
    this in the x dimension and the second describes this in the y dimension
    """
    centerX,centerY=centerLocation
    radLength=sideLength/2

    xMax=centerX+radLength
    xMin=centerX-radLength

    yMin=centerY-radLength
    yMax =centerY+radLength
    return ((xMax,xMin),(yMax,yMin))
def linearMotion(bounds, dimensionalVelocity,time):
    """
    this should describe the locations of the corners after moving with a constant veloctiy for a specified amount of time
    :param bounds: a tuples of the form (dbl,dbl) where the first element is the max and the second element is the min of
    the bounds of a square that is moving in a constant linear speed
    :param dimensionalVelocity: is a dbl that describes how far the bounds move in this direction in a unit time
    :param time: is a dbl it describes the time that the has passed since the test started
    :return: returns the location of the bounds after a specific amount of time
    """
    newBounds=[]
    for side in bounds:
        newBound=side+(dimensionalVelocity*time)
        newBounds.append(newBound)
    return newBounds

def getCricleStart(centerOfSquare,centerOfCircle):
    """
    finds the radius of a circle and describes where the object starts in it's rotation

    ################################
    when using this keep in mind that your center of the circle for a side will be displaced from the center of the
    circle by half of the length of a side
    ################################

    :param centerOfSquare: is a tuple of the form (dbl,dbl) where the first element is the x location of the center and
    the second element is the y location of the center of the square that is moving
    :param centerOfCircle:is a tuple of the form (dbl,dbl) where the first element is the x location of the center and
    the second element is the y location of the center of the circle that the square is moving in
    :return: a tuple of the form (dbl,dbl) where the first element the radius of the circle and the second element is
    the location on the circle in radians
    """
    xdif=centerOfSquare[0]-centerOfCircle[0]
    ydif=centerOfSquare[1]-centerOfCircle[1]
    radius=((xdif)**2+(ydif)**2)**0.5
    #we are gonna use tan to find the side incase radius gets messed up
    angle= math.asin(ydif/radius)

    if not xdif>0:
        angle = math.pi - angle
        if ydif==0:

            if not xPosBool:angle=math.pi
    return (radius,angle)
def cricleStartYComp(angle):
    """
    returns the angle of starting location if in y coordinates
    :param angle: is the angle of the start position in x coordinates
    :return: angle of the start ppostion in y coordinates
    """
    return angle-(math.pi/2)

def speedInRadians(radius, tanSpeed):
    """
    this converts tangential speed to rotational speed in radians
    :param radius: a dbl that describes the radius of the circle that an object is moving around
    :param tanSpeed: a dbl that uses the speed of object along the outside of the circle
    :return: will return the speed in radians
    """
    return tanSpeed/radius

def cicularMotion (bounds,radius,radSpeed,startAngle,time):
    """
    this should describe the location of the upper and lower bound of a dimension of a square after it has been moving
    in a circular motion for some specified time

    ###################################
    this is supposed to work in 1 spacial dimension and time
    ###################################
    :param bounds: is a tuple of the form (dbl,dbl) where the first element is the max and the second element is hte min
    the bounderies of a square that is moving in a circular path
    :param radius: a dbl that describes the radius of the circle
    :param radSpeed: a dbl that describes the speed of the square in radians around the circle
    :param startAngle: a dbl that describes what angle the square is along the circle at time=0
    :param time: a dbl describes the time since the test started
    :return: a tuple in the form (dbl,dbl) where the first element describes the max and the second element describes
    the min of the bounds  of the location of the square at the time entered.
    """

    newCircleLocation=radius*math.cos(startAngle+(time*radSpeed))
    newBounds=[]
    for bound in bounds:
        newBound=bound+newCircleLocation
        newBounds.append(newBound)
    return newBounds
def findDimensionalOverlapTime(cirSqBounds,linSqBounds,startAngle,radSpeed,radius,centerOfCircle,dimensionalVelocity):
    """
    this should return times of possible collision in one spacial dimension for a square moving in a circular motion and
    a square moving with constant linear motion
    :param cirSqBounds: is a tuple of the form (dbl,dbl) where the first element is the max and the second element is
    the min the bounderies of a square that is moving in a circular path
    :param linSqBounds: a tuples of the form (dbl,dbl) where the first element is the max and the second element is the
    min of the bounds of a square that is moving in a constant linear speed
    :param startAngle:  a dbl that describes what angle the square is along the circle at time=0
    :param radSpeed: a dbl that describes the speed of the square in radians around the circle
    :param radius: a dbl that describes the radius of the circle
    :param centerOfCircle: a dbl that describes the location of the center of the circle in this dimension
    :param dimensionalVelocity: a dbl that describes how fast the
    :return: a list of tuples in the form [(dbl,dbl),...] where each tuple has it's first element as the time that the
    overlap of the two squares in this dimension starts and the second element is when it ends. the list will have a
    tuple for each separate incident of overlap
    """


    ##TODO if deminsionalVelocity is 0 we need to figure out how long we run this for
    ##TODO cont do we even run this side or should we have another function
    circleSqEdge= abs((cirSqBounds[0]-cirSqBounds[1]))/2                      #actually describes half of the length of the edge
    circleLimitsMin=centerOfCircle-radius-circleSqEdge
    circleLimitsMax=centerOfCircle+radius+circleSqEdge
    squaresInQuestionTime1=(linSqBounds[0]-circleLimitsMin)/dimensionalVelocity
    squaresInQuestionTime2=(linSqBounds[1]-circleLimitsMax)/dimensionalVelocity
    if squaresInQuestionTime1>squaresInQuestionTime2:
        startTime=squaresInQuestionTime2
        endTime=squaresInQuestionTime1
    else:
        startTime = squaresInQuestionTime1
        endTime = squaresInQuestionTime2

    #here we see where the circle square is at the start
    firstPossibleAngle=(startAngle+(startTime*radSpeed))%(2*math.pi)
    lastPossibleAngle=(startAngle+(endTime*radSpeed))%(2*math.pi)
    firstPossibleCirSquareBounds=cicularMotion(cirSqBounds, radius,radSpeed,startAngle,startTime)
    firstPeakBefore=math.floor((firstPossibleAngle/math.pi))
    isPeakMax=True                          #this describes if the angle of the last peak was at 0 (true) or pi (false)
    if (firstPeakBefore+2)%2==1:
        isPeakMax = False
    ##if it is false we will have the cosine as a negative.
    timeOfPreviousPeak=startTime-(radSpeed*(firstPossibleAngle%math.pi))            #describes the time of the last peak before entering the area where they might interact
    timeofPeakAfterExit=endTime+ (radSpeed*(lastPossibleAngle%math.pi))
    peakTimeIntervals=radSpeed/math.pi                              #describes half of a period
    numOfPasses=int(math.floor(((timeOfPreviousPeak-timeofPeakAfterExit)/peakTimeIntervals)+.5))-1
    #parameters finding the crossing point between the min of circle square and the max of linear square
    cosineParams1=[(centerOfCircle-circleSqEdge),radius,startAngle,radSpeed,linSqBounds[0],dimensionalVelocity]

    #parameters finding the crossing point between the max of circle square and the min of linear square
    cosineParams2=[(centerOfCircle+circleSqEdge),radius,startAngle,radSpeed,linSqBounds[1],dimensionalVelocity]
    #TODO find when we need to check multiple times in  half period,
    #TODO find where we need to check between if multiple times will be


print bisection.derCosineWeights([1,2,3,4,5,6])


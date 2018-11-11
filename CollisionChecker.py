
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
    centerX ,centerY =centerLocation
    radLength =sideLength /2

    xMax =centerX + radLength
    xMin =centerX - radLength

    yMin =centerY - radLength
    yMax =centerY + radLength
    return ((xMax, xMin), (yMax, yMin))


###########################################################################################################
#   Lydia: Fixed this function - position as function of time depends on initial position, not previous position
###########################################################################################################
def linearMotion(startPos, sideLength, dimensionalVelocity, time):  # this function is in 1-D
    """
    this should describe the locations of the corners after moving with a constant veloctiy for a specified amount of time
    :param startPos: dbl describing the starting position of the square in this dimension
    :param sideLength: dbl describing the side length of the square
    :param dimensionalVelocity: is a dbl that describes how far the bounds move in this direction in a unit time
    :param time: is a dbl it describes the time that the has passed since the test started
    :return: returns the location of the bounds after a specific amount of time
    """
    newBounds = []
    newPosition = dimensionalVelocity * time + startPos
    newBound = newPosition + sideLength * 0.5
    newBounds.append(newBound)
    newBound = newPosition - sideLength * 0.5
    newBounds.append(newBound)
    return newBounds


def getCricleStart(centerOfSquare, centerOfCircle):
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
    xdif = centerOfSquare[0] - centerOfCircle[0]
    ydif = centerOfSquare[1] - centerOfCircle[1]
    radius = ((xdif) ** 2 + (ydif) ** 2) ** 0.5
    # we are gonna use tan to find the side incase radius gets messed up
    angle = math.asin(ydif / radius)

    if not xdif > 0:
        angle = math.pi - angle
        if ydif == 0:
            # if not xPosBool:
            angle = math.pi
    return (radius, angle)


def cricleStartYComp(angle):
    """
    returns the angle of starting location if in y coordinates
    :param angle: is the angle of the start position in x coordinates
    :return: angle of the start ppostion in y coordinates
    """
    return angle - (math.pi / 2)


def speedInRadians(radius, tanSpeed):
    """
    this converts tangential speed to rotational speed in radians
    :param radius: a dbl that describes the radius of the circle that an object is moving around
    :param tanSpeed: a dbl that uses the speed of object along the outside of the circle
    :return: will return the speed in radians
    """
    return tanSpeed / radius


###########################################################################################################
#   Lydia: Fixed this function - position as function of time does not depend on previous position
###########################################################################################################
def circularMotion(centerOfCircle, sideLength, radius, radSpeed, startAngle, time):  # this function is in 1-D
    """
    this should describe the location of the upper and lower bound of a dimension of a square after it has been moving
    in a circular motion for some specified time

    ###################################
    this is supposed to work in 1 spacial dimension and time
    ###################################
    :param centerOfCircle: dbl describing location of center of circle in this dimension
    :param sideLength: dbl describing the side length of the square
    :param radius: a dbl that describes the radius of the circle
    :param radSpeed: a dbl that describes the speed of the square in radians around the circle
    :param startAngle: a dbl that describes what angle the square is along the circle at time=0
    :param time: a dbl describes the time since the test started
    :return: a tuple in the form (dbl,dbl) where the first element describes the max and the second element describes
    the min of the bounds  of the location of the square at the time entered.
    """

    newCircleLocation = centerOfCircle + radius * math.cos(startAngle + (time * radSpeed))
    newBounds = []
    newBound = newCircleLocation + (sideLength * 0.5)
    newBounds.append(newBound)
    newBound = newCircleLocation - (sideLength * 0.5)
    newBounds.append(newBound)
    return newBounds


def findDimensionalOverlapTime(cirSqBounds, linSqBounds, startAngle, radSpeed, radius, centerOfCircle,
                               dimensionalVelocity):
    """
    this should return times of possible collision in one spacial dimension for a square moving in a circular motion and
    a square moving with constant linear motion
    :param cirSqBounds: is a tuple of the form (dbl,dbl) where the first element is the max and the second element is
    the min the bounderies of a square that is moving in a circular path
    :param linSqBounds: a tuples of the form (dbl,dbl) where the first element is the max and the second element is the
    min of the bounds of a square that is moving in a constant linear speed
    :param startAngle:  a dbl that describes what angle the square is along the circle at time=0 assumes we are in X (add pi/2 for y)
    :param radSpeed: a dbl that describes the speed of the square in radians around the circle
    :param radius: a dbl that describes the radius of the circle
    :param centerOfCircle: a dbl that describes the location of the center of the circle in this dimension
    :param dimensionalVelocity: a dbl that describes how fast the
    :return: a list of tuples in the form [dbl,...] where each dbl is a start or end of an overlap. the first element
     should be the start of an overlap
    """
    ##########################

    ##########################

    errorTolerence = .000001

    ##########################

    ##########################


    circleSqEdge = abs((cirSqBounds[0] - cirSqBounds[1])) / 2  # actually describes half of the length of the edge

    circleLimitsMin = centerOfCircle - radius - circleSqEdge  # lower limit of where collision is possible
    circleLimitsMax = centerOfCircle + radius + circleSqEdge  # upper limit of where collision is possible

    squaresInQuestionTime1 = (circleLimitsMin - linSqBounds[
        0]) / dimensionalVelocity  # how long it will take A to reach lower limit
    squaresInQuestionTime2 = (circleLimitsMax - linSqBounds[
        1]) / dimensionalVelocity  # how long it will take A to reach upper limit

    if squaresInQuestionTime1 > squaresInQuestionTime2:  # A is moving from positive to negative
        startTime = squaresInQuestionTime2
        endTime = squaresInQuestionTime1
    else:  # A is moving from negative to positive
        startTime = squaresInQuestionTime1
        endTime = squaresInQuestionTime2

    # here we see where the circle square is at the start
    #print "start time", startTime

    firstPossibleAngle = (startAngle + (startTime * radSpeed)) % (2 * math.pi)
    #print"start angle", firstPossibleAngle
    lastPossibleAngle = (startAngle + (endTime * radSpeed)) % (2 * math.pi)
    #print"last angle", lastPossibleAngle

    ## Lydia: Changed function call to work with fixed circularMotion function ##
    firstPossibleCirSquareBounds = circularMotion(centerOfCircle, circleSqEdge * 2, radius, radSpeed, startAngle,
                                                  startTime)

    firstPeakBefore = math.floor(
        (firstPossibleAngle / math.pi))  # first cosine extremum before A enters range of B (is 0 for max, 1 for min)

    ##if it is false we will have the cosine as a negative.

    ###############################################################
    # Lydia: fixed math errors
    ###############################################################
    timeOfPreviousPeak = startTime - ((
                                                  firstPossibleAngle % math.pi) / radSpeed)  # describes the time of the last peak before entering the area where they might interact
    timeofPeakAfterExit = endTime + (
                (math.pi - (lastPossibleAngle % math.pi)) / radSpeed)  # time of extremum after A exits range of B
    peakTimeIntervals = math.pi / radSpeed  # time of half of a period
    """
    """
    #################################################################################
    ## Lydia: Removed the -1 from the end of the computation for numOfPasses because
    ##        it cuts off the last half-period that might contain crossings
    #################################################################################
    numOfPasses = int(math.floor(((timeofPeakAfterExit - timeOfPreviousPeak) / peakTimeIntervals) + .5))

    # parameters finding the crossing point between the min of circle square and the max of linear square
    cosineParams1 = [(centerOfCircle - circleSqEdge), radius, startAngle, radSpeed, linSqBounds[0], dimensionalVelocity]

    # parameters finding the crossing point between the max of circle square and the min of linear square
    cosineParams2 = [(centerOfCircle + circleSqEdge), radius, startAngle, radSpeed, linSqBounds[1], dimensionalVelocity]
    # the derivative of both of thesee will be the same.
    derCosineParams = bisection.derCosineWeights(cosineParams1)
    maxDerivCos = derCosineParams[1]  # ask Thomas about this
    zerosLin = []

    print("numOfPasses", numOfPasses)

    for i in range(0, numOfPasses):

        isPeakMax = True  # this describes if the angle of the last peak was at 0 (true) or pi (false)
        """
        #debug info for peak times
        print("peaktimestuff")
        print(timeOfPreviousPeak)
        print(peakTimeIntervals)
        """
        ########################################################
        # Lydia: Debugged and rewrote condition to set whether
        #        isPeakMax is True or False
        #
        #        In the new version, 1 is true, 0 is false
        ########################################################
        if firstPeakBefore == 0:
            isPeakMax = (i + 1) % 2
        else:
            if firstPeakBefore == 1:
                isPeakMax = i % 2

        #print("isPeakMax", isPeakMax)

        if isPeakMax:  # current derivative goes down
            if -maxDerivCos < dimensionalVelocity and dimensionalVelocity < 0:
                q = 1

                # TODO find where we need to check between for possible multiple crossings
                # TODO implement a method that works like below with the checks as bounds
                # we can do this by finding the roots of the derivative and you check from the first peak to the first 0,
                # from the first 0 to the second 0, then from the second 0 to the last peak.
            else:
                ## Lydia: Changed (i-1) to (i+1) so correct half-period would be checked instead of previous half-period ##

                if bisection.cosineFunction((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                            cosineParams1) * bisection.cosineFunction(
                        (timeOfPreviousPeak + peakTimeIntervals * (i)), cosineParams1) <= 0:
                    zero, _ = bisection.newtonMethod((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                                     (timeOfPreviousPeak + peakTimeIntervals * i), cosineParams1, 0,
                                                     errorTolerence, derCosineParams)
                    zerosLin.append(zero)

                ## Lydia: Corrected newtonMethod function call to use cosineParams2 instead of cosineParams1 ##   
                if bisection.cosineFunction((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                            cosineParams2) * bisection.cosineFunction(
                        (timeOfPreviousPeak + peakTimeIntervals * (i)), cosineParams2) <= 0:
                    zero, _ = bisection.newtonMethod((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                                     (timeOfPreviousPeak + peakTimeIntervals * i), cosineParams2, 0,
                                                     errorTolerence, derCosineParams)
                    zerosLin.append(zero)

        else:
            if maxDerivCos > dimensionalVelocity and dimensionalVelocity > 0:
                # TODO find where we need to check between for possible multiple crossings
                # TODO implement a method that works like below with the checks as bounds
                q = 1
            else:
                ## Lydia: Changed (i-1) to (i+1) so correct half-period would be checked instead of previous half-period ##

                if bisection.cosineFunction((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                            cosineParams1) * bisection.cosineFunction(
                    (timeOfPreviousPeak + peakTimeIntervals * (i)), cosineParams1) <= 0:
                    zero, _ = bisection.newtonMethod((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                                     (timeOfPreviousPeak + peakTimeIntervals * i), cosineParams1, 0,
                                                     errorTolerence, derCosineParams)
                    zerosLin.append(zero)

                ## Lydia: Corrected newtonMethod function call to use cosineParams2 instead of cosineParams1 ##     
                if bisection.cosineFunction((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                            cosineParams2) * bisection.cosineFunction(
                    (timeOfPreviousPeak + peakTimeIntervals * (i)), cosineParams2) <= 0:
                    zero, _ = bisection.newtonMethod((timeOfPreviousPeak + peakTimeIntervals * (i + 1)),
                                                     (timeOfPreviousPeak + peakTimeIntervals * i), cosineParams2, 0,
                                                     errorTolerence, derCosineParams)
                    zerosLin.append(zero)

    return zerosLin
def findOverlapIntNoDimVelocity(cirSqBounds, linSqBounds, startAngle, radSpeed, radius, centerOfCircle, endTime):

    """
    this function finds overlap times if the dimensional velocity is 0 and must be implemented with a time to end the loop so that it resolves.

    :param cirSqBounds: is a tuple of the form (dbl,dbl) where the first element is the max and the second element is
    the min the bounderies of a square that is moving in a circular path
    :param linSqBounds: a tuples of the form (dbl,dbl) where the first element is the max and the second element is the
    min of the bounds of a square that is moving in a constant linear speed
    :param startAngle:  a dbl that describes what angle the square is along the circle at time=0
    :param radSpeed: a dbl that describes the speed of the square in radians around the circle
    :param radius: a dbl that describes the radius of the circle
    :param centerOfCircle: a dbl that describes the location of the center of the circle in this dimension
    :param endTime: is the time that we check until for
    :return: a list of tuples in the form [(dbl,dbl),...] where each tuple has it's first element as the time that the
    overlap of the two squares in this dimension starts and the second element is when it ends. the list will have a
    tuple for each separate incident of overlap it may be an empty list this should be handled pretty well for the program as is (11/11/2018 13,37EST).
    """
    ##########################

    ##########################

    errorTolerence = .000001

    ##########################

    ##########################

    #first define functions and variables we need for the 2 half period checks
    circleSqEdge = abs((cirSqBounds[0] - cirSqBounds[1])) / 2  # actually describes half of the length of the edge

    circleLimitsMin = centerOfCircle - radius - circleSqEdge
    circleLimitsMax = centerOfCircle + radius + circleSqEdge




    #next check if the square passes through the cricle at all if it doesn't then retrun an empty list
    if cirSqBounds[0]<circleLimitsMin or cirSqBounds[1]<circleLimitsMax :
        return []               #how very pythonic

    #we then find all overlap starts and end over 2 half periods and from now on treat them like a whole period
    #we next check if the square started in an overlap or not
    #   if overlap: we say the start of the first overlap is 0 and the second is the first time keep adding times til we are out of them then finish with the first time
    #   else: we say the first time is the start of the first overlap carry on the same way end with the last crossing time.
    #append list with times +N*period til the end overlap time is after the end time.
    #return list



def collisionTimeFromOverlaps(lstXOverlaps, lstYOverlaps):
    """
    this function will find the collision time if you input the times where objects overlap in the x direction and where
     they overlap in the y directions
    :param lstXOverlap: a list of tuples in the form [(dbl,dbl),...] where each tuples describes an overlap and the
    first dbl in the tuple is the begining time of an overlap and the second dbl is the time that an overlap ends in the
    x dimension.
    :param lstYOverlap: a list of tuples in the form [(dbl,dbl),...] where each tuples describes an overlap and the
    first dbl in the tuple is the begining time of an overlap and the second dbl is the time that an overlap ends in the
    y dimension.
    :return: a dbl that describes the first overlap time, or None if no overlap is detected.
    """

    earliestCollision = float('inf')
    for xOverlap in lstXOverlaps:
        for yOverlap in lstYOverlaps:
            if xOverlap[0] > yOverlap[0] and xOverlap[0] < yOverlap[1]:
                earliestCollision = min(earliestCollision, xOverlap[0])

            if yOverlap[0] > xOverlap[0] and yOverlap[0] < xOverlap[1]:
                earliestCollision = min(earliestCollision, yOverlap[0])
    if earliestCollision == float('inf'):
        earliestCollision = None
    return earliestCollision


"""
file input
"""

file1 = open("input.txt")
lines = file1.read().split('\n')
line1 = lines[1].split(' ')
line2 = lines[2].split(' ')
linEdge = float(line1[0])
cirEdge = float(line2[0])
linX = float(line1[1])
cirX = float(line2[1])
linY = float(line1[2])
cirY = float(line2[2])
linVeloX = float(line1[3])
centerX = float(line2[3])
centerY = float(line2[4])
linVeloY = float(line1[4])
cirLinSpeed = float(line2[5])

print(linEdge, linX, linY, linVeloX, linVeloY)
print(cirEdge, cirX, cirY, centerX, centerY, cirLinSpeed)

print "here we start the angle check."
squarePoss = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
for pos in squarePoss:
    print "circle pos", pos, (getCricleStart(pos, (0, 0)))

print "here we end the angle check."

print "here we start checking collision from overlap"
lstTestCases = []
# first we check for a list with 1 collision ;expected output: 4 :pass
lstTestCases.append(([(4.0, 5.0)], [(2.0, 4.5)]))
# check for a list with no collision; expected output:None : pass
lstTestCases.append(([(1.0, 2.0)], [(0.2, 0.5), (3.0, 5.2)]))
# check for a list with multiple collision; expected output:2.0 :pass
lstTestCases.append(([(1.40, 2.1), (3.2, 5.3), (6.4, 7.8)], [(2.0, 3.5), (4.0, 7.2)]))
# check for an empty list: expected output: None :pass
lstTestCases.append(([], [(2.0, 3.5), (4.0, 7.2)]))
for testCase in lstTestCases:
    xOverlap = testCase[0]
    yOverlap = testCase[1]
    print collisionTimeFromOverlaps(xOverlap, yOverlap)

print"here we stop checking collision from overlap"
linearStartLocation = (linX, linY)
cirSquareStartLoc = (cirX, cirY)
linearSideLength = linEdge
cirSquareSideLength = cirEdge
cirSpeed = cirLinSpeed
dimensionalXVelocity = linVeloX
centerOfCir = (centerX, centerY)
cirSquareStartBoundsx, cirSquareStartBoundsy = findSidesAtStart(cirSquareStartLoc, cirSquareSideLength)
radius, startAngle = getCricleStart(cirSquareStartLoc, centerOfCir)
radSpeed = speedInRadians(radius, cirSpeed)

xBounds, yBounds = findSidesAtStart(linearStartLocation, linearSideLength)
findDimensionalOverlapTime(cirSquareStartBoundsx, xBounds, startAngle, radSpeed, radius, centerOfCir[0],
                           dimensionalXVelocity)

import math
import datetime
def definePolyFunction():
    """
    creates a polynomial functions parameters       called as function to keep code clean
    :return: (list) a list of the polynomial parameters with the x^0 at location 0 and
    counting up from there
    """
    lstWeights=[]
    degree = input("degree of polynomial in terms of highest exponent of x:")
    degree = int(degree+1)
    for a in range (0,degree):
        string='weight for x^'+str(a)+':'
        weight = input(string)
        weight = float(weight)
        lstWeights.append(weight)
    return lstWeights
def polyFunction(x,weights):
    """
    evaluates a polynomial at a given x     called as funtion to limit number of times i need to type this
    :param x: (float)is where a polynomial is evaluated at
    :param weights: (list) of the polynomial parameters with the x^0 at location 0 and
    counting up from there
    :return: (float) the value of the polynomial function at x
    """
    y=0
    for i in range (0,len(weights)):
        y+= weights[i]*(x**i)
    return y
def guessChecker(polyWeights):
    """
    inputs and checks if guesses are valid made this a function because it is recursive
    :param polyWeights: is the weights of the function that we are evaluating in
    :return: a high guess and a low guess
    """

    highGuess = input("input your high guess:")
    highGuess = float(highGuess)
    lowGuess = input("input your low guess:")
    lowGuess = float(lowGuess)
    if (polyFunction(highGuess, polyWeights) * polyFunction(lowGuess, polyWeights) > 0):
        print "The guesses are on the same side of 0. let's try again if you want to exit hit ctrl+C"
        highGuess,lowGuess=guessChecker(polyWeights)
    return highGuess,lowGuess
def bisectionMethod(highGuess,lowGuess,polyWeights,iterations,isHighGuessEstimate,errorTolerence):
    """

    :param iterations: should be the number of times that this has been called before
    :param highGuess: (float) a guess
    :param lowGuess:  (float) a guess that is on the other side of the zero
    :param polyWeights: (list) a list of the polynomial parameters with the x^0 at location 0 and
    counting up from there
    :param isHighGuessEstimate: will be true if highGuess was the estimate of the last iteration and false if it was the lowGuess
    :return: returns a tup of the form (float,int) where the first is the estimate of the zero and the second
    is the number of iterations that this required
    """
    estimate = (highGuess + lowGuess) / 2
    number=(cosineFunction(highGuess, polyWeights) * cosineFunction(estimate, polyWeights) )
    if (number==0):
        if (cosineFunction(highGuess, polyWeights)==0):
            return estimate, iterations
        return highGuess, iterations
    if isHighGuessEstimate:
        error=abs((polyFunction(highGuess, polyWeights)-polyFunction(estimate, polyWeights))/polyFunction(estimate, polyWeights))
    else:
        error = abs((polyFunction(lowGuess, polyWeights) - polyFunction(estimate, polyWeights)) / polyFunction(estimate,polyWeights))
    if error<errorTolerence:

        return estimate, iterations
    if (number> 0):
        return bisectionMethod(estimate,lowGuess,polyWeights,(iterations+1),True,errorTolerence)
    elif(number< 0):
        return bisectionMethod(highGuess, estimate, polyWeights, (iterations + 1),False,errorTolerence)
    return None
def newtonMethod(highGuess,lowGuess,polyWeights,iterations,errorTolerence):

    """

    :param highGuess:
    :param lowGuess:
    :param polyWeights:
    :param iterations:
    :param errorTolerence:
    :return:
    """

    derCosW=derCosineWeights(polyWeights)
    estimate = (highGuess + lowGuess) / 2
    newGuess=estimate-(cosineFunction(estimate,polyWeights)/cosineFunction(estimate,derCosW))
    error=abs((highGuess-newGuess)/newGuess)
    if error<errorTolerence:
        return estimate,iterations
    else:
        return newtonMethod(newGuess, newGuess,polyWeights,iterations+1,errorTolerence)
def cosineFunction(x,parameters):

    """
    
    :param x: where we evaluate the funtion at
    :param parameters: list of parameters of a cosine funtion of the form below
    :return: the value of a cosine function of the form below
    """
    return parameters[0]+parameters[1]*math.cos(parameters[2]+parameters[3]*x)-(parameters[4]+parameters[5]*x)
def derCosineWeights(parameters):
    """
    
    :param parameters: 
    :return: parameters of the derviative of the cosine funtion
    """
    par1=(parameters[1]*parameters[3])
    par2=(parameters[2]+math.pi/2)
    return [0, par1 ,par2,parameters[3],parameters[5],0]
#######################

#######################
#you can change this to change error tolerence it will be in terms of a value not a percentage
errorTolerence=.000000000000001

#######################

#######################

#polyWeights=definePolyFunction()
testWeights=[[0,1.3,.2,1,-.5,1.3],[.2,.6,-.3,2,0,1.3],[-.3,.89,.31,1,.02,.96],[23,15,6,.43,40,-12],[-14,32,2,2,14,-2.4]]
for polyWeights in testWeights:
    print "parameters for the next methods:",polyWeights
    highGuess,lowGuess=(0,math.pi)
    start=datetime.datetime.now()
    root,i=bisectionMethod(highGuess, lowGuess, polyWeights, 0, True, errorTolerence)
    end=datetime.datetime.now()
    duration=end-start

    print "Using Bisection Method:This is the root:",root," It took",i,"steps to get there. and it took:",duration

    start=datetime.datetime.now()
    root,i=newtonMethod(highGuess, lowGuess, polyWeights, 0, errorTolerence)
    end = datetime.datetime.now()
    duration = end - start

    print "Using Newtons Method:This is the root:",root," It took",i,"steps to get there.and it took:",duration


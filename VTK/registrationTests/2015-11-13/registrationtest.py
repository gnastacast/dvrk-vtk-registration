import registration
import vtk
import os
import time

def iterateStates(filepath):
	with open(filepath,"r") as fh:
		l = [ map(str,line.split('\n')) for line in fh]
	for idx in range(0,len(l)):
		s = map(float, l[idx][0].split(' '))
		outputTransform = vtk.vtkTransform()
		outputTransform.PostMultiply()
		outputTransform.RotateY(s[4])
		outputTransform.RotateX(s[3])
		outputTransform.RotateZ(s[5])
		outputTransform.Translate(s[0],s[1],s[2])
		l[idx] = outputTransform;
	return l

def iterateTransforms(transforms, filepath) :
	text = ""
	for idx in range(0,len(transforms)) :
		# Turn transform into state vector
		pos = transforms[idx].GetPosition()
		rot = transforms[idx].GetOrientation()
		s = [pos[0],pos[1],pos[2],rot[0],rot[1],rot[2]]
		for stateIdx in range(0,len(s)-1) :
			text += str(s[stateIdx]) + " "
		text += str(s[len(s)-1])+'\n'
	print("Saving state to: %s" % filepath)
	with open(filepath, "w") as fh:
		fh.write(text)

manualIterations = 0
manualRegs = []
actualRegs = []
for idx in range(0,manualIterations) :
	print('registration number ' + str(idx+1))
	reg = registration.RegistrationObject('suzanne.stl')
	actualRegs = actualRegs+[reg.actualTransform]
	manualRegs = manualRegs+[reg.manualRegistration()]
	os.rename("bw.png", "registrationTests/bw_"+str(1000+idx)+".png")

#iterateTransforms(manualRegs,"registrationTests/reg_manual.txt")
#iterateTransforms(actualRegs,"registrationTests/reg_actual.txt")

manualRegs = iterateStates("registrationTests/reg_manual.txt")

annealedIterations = 50
annealRegs = []
times = []
for outerIdx in range(0,len(manualRegs)) :
	for innerIdx in range(0,annealedIterations) :
		print('registration # ' + str(outerIdx) +", iteration # " +str(innerIdx))
		reg = registration.RegistrationObject('suzanne.stl',bwImage = "registrationTests/bw_"+str(1000+outerIdx)+".png", colorImage="color.png")
		startTime = time.clock()
		annealRegs = annealRegs + [reg.annealingRegistration(manualRegs[outerIdx])]
		times = times + [time.clock()-startTime]
timesTotal = 0
for idx in range(0,len(times)):
	timesTotal+=times[idx]
print('Average time: ' + str(timesTotal/len(times)))
iterateTransforms(annealRegs,"registrationTests/reg_anneal.txt")

'''
Manual registration data

-4.98325571605 -21.037167066 -125.735833179 79.5073900278 -146.22979591 -129.21334215
-22.1885808418 32.3881835715 -174.432977332 -1.32010202288 -148.578270047 75.580373982
6.62562981133 5.03184012518 -169.57899089 33.3250878135 142.74270065 -12.4822383704
6.7342803096 47.1801898803 -150.042110745 73.3166246719 143.552964464 125.104452324
40.9839671207 36.7961741877 -124.437875436 -69.4897754065 -171.530898447 -17.6932668329

Actual registration data

-4.42525688376 -20.2198267934 -121.958155675 78.7403760776 -133.522420804 -141.14067324 
-21.5105030998 32.1255700509 -172.119449103 -2.87592545266 -149.228391794 75.8538079359 
6.69363073429 5.14150457437 -176.446493954 31.6778550836 145.533938456 -14.5117233298 
6.66559486626 47.580003101 -148.838499337 73.554424415 143.498876286 124.14057807 
40.8730957531 36.6210111266 -119.623043018 -69.3364216515 -174.875592768 -21.4560932265 

'''

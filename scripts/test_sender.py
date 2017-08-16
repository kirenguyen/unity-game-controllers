
def try_graph():
	#!/usr/bin/python
	import PyQt5
	import matplotlib.pyplot as plt
	import time

	PyQt5.QtCore.pyqtRestoreInputHook()

	def barlist(n): 
	    return [1/float(n*k) for k in range(1,100)]

	fig=plt.figure()

	n=1000 #Number of frames
	i=1
	x=range(i,100)
	barcollection = plt.bar(x,barlist(1))

	while True:
		fig.canvas.flush_events()
		i+=1
		time.sleep(1)
		plt.show(block=False)




if __name__ == '__main__':
    try_graph()

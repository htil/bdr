#!/usr/bin/env python
import Bebop

if __name__ == '__main__':
	drone = Bebop()
	
	while True:
		drone.takeoff()
		
		while not drone.done:
			drone.move()
			
		drone.land()
			
		raw_input("Press enter to race again")

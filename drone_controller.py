from dronekit import connect, VehicleMode ,Vehicle , LocationLocal , LocationGlobal, LocationGlobalRelative
import time

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

try:
    def battery_check():
        if(vehicle.battery < 9.9):
            print ("Battery Low. Landing")
            print ("Battery: %s" % vehicle.battery)
            
        else:
            print ("Battery: %s" % vehicle.battery)
        
    
    def arm_and_takeoff(aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print ("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not vehicle.is_armable:
            print (" Waiting for vehicle to initialise...")
            time.sleep(1)

        print ("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode    = VehicleMode("GUIDED")
        vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print (" Waiting for arming...")
            time.sleep(1)

        print ("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print (" Altitude: ", vehicle.location.global_relative_frame.alt)
            battery_check()
            #Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print ("Reached target altitude")
                break
            time.sleep(1)

    arm_and_takeoff(10)
    vehicle.mode = VehicleMode("LAND")

except KeyboardInterrupt:
    vehicle.armed= False

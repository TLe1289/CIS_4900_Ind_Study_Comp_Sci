"""drive_my_robot controller."""

#Make sure this Robot has the "Supervisor" checkmark[]
from controller import Supervisor
from controller import Camera

if __name__ == "__main__":
    
    # create the Robot instance.
    robot = Supervisor()
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
 
    left_motor = robot.getDevice('motor_1')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    right_motor = robot.getDevice('motor_2')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0)
    
    camera = robot.getDevice('CAM')
    camera.enable(timestep)
    camera.recognitionEnable(timestep)
 
    #root_node = robot.getFromDef('Robot MK2')
    #child = root_node.getField('fuck it this shit shouldnt work')
    #count= robot.getRoot().getNumberOfFields()
    #translation_field = root_node.getField("translation")
    left_motor.setVelocity(1)
    right_motor.setVelocity(-1)
    while robot.step(timestep) != -1:
        count = camera.getRecognitionNumberOfObjects()
        #print(count)
        if count > 0:
            object = camera.getRecognitionObjects()[0]
            
            #print("X:{} Y:{} Z:{}".format(object.getPosition()[0], object.getPosition()[1],object.getPosition()[2]))
            print(object.getPosition()[1])
            
            if (object.getPosition()[1] <.02 and object.getPosition()[1]>-.02 ):
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
            
        #if -.02<object.getPosition()[1]and object.getPosition()[1]< .02:
        left_motor.setVelocity(5)
        right_motor.setVelocity(5)
        #image = camera.getImage()
        #print(translation_field[0])
        #print(coordinates[1])
        
        


        pass
    
    
    
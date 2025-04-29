"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
 # from controller import Robot, Motor, DistanceSensor
# from controller import Robot

# create the Robot instance.
# robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
 # motor = robot.getDevice('motorname')
 # ds = robot.getDevice('dsname')
 # ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
     # val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
     # motor.setPosition(10.0)
    # pass

# Enter here exit cleanup code.
from controller import Supervisor

supervisor = Supervisor()
# Tempo de passo da simulação
timestep = int(supervisor.getBasicTimeStep())

left_motor = supervisor.getDevice("left wheel motor")
right_motor = supervisor.getDevice("right wheel motor")

ds0 = supervisor.getDevice("ps0")  
ds1 = supervisor.getDevice("ps1") 
ds2 = supervisor.getDevice("ps2")
ds3 = supervisor.getDevice("ps3")  
ds4 = supervisor.getDevice("ps4") 
ds5 = supervisor.getDevice("ps5") 
ds6 = supervisor.getDevice("ps6") 
ds7 = supervisor.getDevice("ps7") 
 

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ds0.enable(timestep)
ds1.enable(timestep)
ds2.enable(timestep)
ds3.enable(timestep)
ds4.enable(timestep)
ds5.enable(timestep)
ds6.enable(timestep)
ds7.enable(timestep)


bShouldTurnRight = True
bIsTurning = False

maxDistance = 110

while supervisor.step(timestep) != -1:

    caixa_node = supervisor.getFromDef("C0")
    if caixa_node is None:
        print("Erro: Não foi possível encontrar a caixa.")
        exit()

    # Acessa o campo de tradução (posição) da caixa
    translation_field = caixa_node.getField("translation")
    caixa_pos = translation_field.getSFVec3f()  # retorna [x, y, z]
    print(f"Posição da caixa: {caixa_pos}")

    distance0 = ds0.getValue()
    distance1 = ds1.getValue()
    distance2 = ds2.getValue()
    distance3 = ds3.getValue()
    distance4 = ds4.getValue()
    distance5 = ds5.getValue()
    distance6 = ds6.getValue()
    distance7 = ds7.getValue()
    
    if (distance1 < maxDistance and distance2 < maxDistance*0.9 
        and distance5 < maxDistance*0.9 and distance6 < maxDistance
        and distance7 < maxDistance and distance0 < maxDistance
        and ((not bShouldTurnRight and distance3 < maxDistance*0.9) 
        or (bShouldTurnRight and distance4 < maxDistance*0.9))):
        
        left_motor.setVelocity(6.28)
        right_motor.setVelocity(6.28)
        
        bIsTurning = False     
        
    else:
        
        print("------------------------")
        if(distance0 > maxDistance): print("D0")
        if(distance1 > maxDistance): print("D1")
        if(distance2 > maxDistance*0.9): print("D2")
        if(distance3 > maxDistance*0.9): print("D3")
        if(distance4 > maxDistance*0.9): print("D4")
        if(distance5 > maxDistance*0.9): print("D5")
        if(distance6 > maxDistance): print("D6")
        if(distance7 > maxDistance): print("D7") 
        print("------------------------")
    
        if(not bIsTurning):
            bIsTurning = True
            bShouldTurnRight = not bShouldTurnRight
        if(bShouldTurnRight):
            left_motor.setVelocity(4)
            right_motor.setVelocity(-0.5)
        else:
            left_motor.setVelocity(-0.5)
            right_motor.setVelocity(4)     
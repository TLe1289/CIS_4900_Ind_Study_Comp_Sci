from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
bb8_node = robot.getRoot()
translation_field = bb8_node.getField('translation')

i = 0
while robot.step(TIME_STEP) != -1:
  # [CODE PLACEHOLDER 2]
    if i == 20:
        new_value = [2.5, 0, 0]
        translation_field.setSFVec3f(new_value)
    i += 1
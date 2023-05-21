# max_ball_confidence = 0
# max_blue_bot_confidence =[1]*6  
# max_yellow_bot_confidence =[0.0,0.0,0.0,0.0,0.0,0.0]
# #max_blue_bot_confidence[5]=0.5
# #max_blue_bot_confidence.append(0.001)
# print(max_blue_bot_confidence[5])
# num_cam=10
# use_grsim_vision=0
# port = 10020 if(use_grsim_vision) else 10006
# print(port)
# def a():
#     num_cam=0
#     print(num_cam)
import math
# a()
data= {
  "detection": {
    "frame_number": 377308,
    "t_capture": 13977.820277,
    "t_sent": 1505683703.172275,
    "camera_id": 1,
    "balls": [
    #   {
    #     "confidence": 0.930344820022583,
    #     "area": 78,
    #     "x": -14.979391098022461,
    #     "y": -12.432811737060547,
    #     "z": 0,
    #     "pixel_x": 276.5769348144531,
    #     "pixel_y": 225.73077392578125
    #   }
    #   "{... other balls ...}"
    ],
    "robots_yellow": [
      {
        "confidence": 0.930344820022583,
        "area": 78,
        "x": -14.979391098022461,
        "y": -12.432811737060547,
        "z": 0,
        "pixel_x": 276.5769348144531,
        "pixel_y": 225.73077392578125
      }
    #   "{... other yellow robots ...}"
    ],
    "robots_blue": [
      {
        "confidence": 0.930344820022583,
        "area": 78,
        "x": -14.979391098022461,
        "y": -12.432811737060547,
        "z": 0,
        "pixel_x": 276.5769348144531,
        "pixel_y": 225.73077392578125
      }
    #   "{... other blue robots ...}"
    ]
  }
}
# if data.HasField('detection'):
#     print(data.detection)

print(len(data["detection"]["balls"]))

a = (100*math.cos(30.0*math.pi/180.0))/(0.025 * math.pi)
print(126+((a-5000.0)*126)/5000.0)
